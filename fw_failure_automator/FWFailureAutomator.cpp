#include "FWFailureAutomator.hpp"
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <vector>
#include <random>
#include <functional>
#include <algorithm>
#include <string.h>
#include <random>

using math::constrain;
using std::vector;
using std::pair;
using std::sort;
using std::rand;
using std::default_random_engine;
using std::normal_distribution;


float EPSILON = 0.000001;
inline bool float_eq(float a, float b){
	return (a - b)*(a - b) < EPSILON;
}

inline void fwfa_namespace::_make_switch_topic_inst_cmd(uint8_t inst, vehicle_command_s *cmd)
{
	cmd->timestamp = hrt_absolute_time();
	cmd->command = vehicle_command_s::VEHICLE_CMD_SWITCH_UORB_TOPIC_INST;
	cmd->param1 = static_cast<float>(*ORB_ID(actuator_controls_0).o_id); //target topic
	cmd->param2 = static_cast<float>(inst); // instance to switch to
}

float fwfa_namespace::_get_gausserror(float mean, float sdev)
{
	// float u1 =  rand() / ((float) RAND_MAX); //uniform(0,1] random floats
	// float u2 = rand() / ((float) RAND_MAX);
	// float randStdNormal = sqrt(-2.0f * (float) log(u1)) *
	// 		      (float) sin(2.0f * (float)MATH_PI * u2); //random normal(0,1)
	// return mean + sdev * randStdNormal; //random normal(mean, stdDev^2)

	default_random_engine generator;
    	normal_distribution<float> dist(mean, sdev);
	return dist(generator);
}

uint8_t fwfa_namespace::_sample_from_dist(float* dist, size_t dist_len){
	std::vector<std::pair<uint8_t, float>> state_prob_pairs;

	for (uint8_t i = 0; i < dist_len; i++) { state_prob_pairs.push_back(std::make_pair(i, dist[i])); }

	// sort states by associated probs
	std::sort(state_prob_pairs.begin(), state_prob_pairs.end(),
		  [](std::pair<uint8_t, float> p1, std::pair<uint8_t, float> p2)
	{return p1.second < p2.second;}
	);

	// convert pair vector to CDF
	float sum = 0;
	for (uint8_t i = 0; i < NUM_STATES; i++) {
		sum += state_prob_pairs[i].second;
		state_prob_pairs[i].second = sum;
	}
	// CDF Method
	float r = rand() / ((float) RAND_MAX); //uniform[0,1] random float
	uint8_t i = 0;
	while (!(state_prob_pairs[i].second < r && r <= state_prob_pairs[i + 1].second)) {i++;}

	return state_prob_pairs[i+1].first; //i always stops 1 before
}



using namespace fwfa_namespace;

FWFailureAutomator::FWFailureAutomator(): ModuleParams(nullptr) {}

FWFailureAutomator::~FWFailureAutomator()
{
	PX4_WARN("FWFA destructor called");
}

void FWFailureAutomator::build_fmode_vec()
{
	//defaults to link failure
	failure_mode_t fvec[5] = {
		static_cast<failure_mode_t>(_param_lail_fmode.get()),
		static_cast<failure_mode_t>(_param_rail_fmode.get()),
		static_cast<failure_mode_t>(_param_ele_fmode.get()),
		static_cast<failure_mode_t>(_param_thr_fmode.get()),
		static_cast<failure_mode_t>(_param_rud_fmode.get())
	};

	//confirms we changed the fmode for an actuator
	for (uint8_t i = 0; i < 5; i++) {
		if (fvec[i] != fmode_param_vec[i]) { PX4_INFO("set fmode param %d from %d to %d", i, fmode_param_vec[i], fvec[i]); }
	}

	memcpy(&fmode_param_vec, fvec, sizeof(fvec));

	PX4_INFO("FWFA built fmode_vec from params");
}

void FWFailureAutomator::build_ttable(float loopback_prob, float groundstate_prob)
{
	float lp = loopback_prob;
	float gp = groundstate_prob;
	float e_1 = (1.0f - (lp + gp)); //1 edge
	float e_2 = (1.0f - (lp + gp)) / 2.0f; //2 edges
	float e_3 = (1.0f - (lp + gp)) / 3.0f; //3 edges
	float g_e = (1.0f - lp) / 4.0f; //groundstate edges (groundstate prob is just loopback prob)

	// temp_table will deallocate from stack when we leave this scope
	float temp_table[NUM_STATES][NUM_STATES]  = {
		{lp, g_e, g_e, g_e, g_e, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, //NO_FAIL
		{gp, lp, 0.0f, 0.0f, 0.0f, 0.0f, e_3, e_3, e_3, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, //LAIL
		{gp, 0.0f, lp, 0.0f, 0.0f, 0.0f, e_3, e_3, e_3, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, //RAIL
		{gp, 0.0f, 0.0f, lp, 0.0f, 0.0f, e_2, 0.0f, 0.0f, 0.0f, e_2, 0.0f, 0.0f, 0.0f, 0.0f}, //ELE
		{gp, 0.0f, 0.0f, 0.0f, lp, 0.0f, 0.0f, e_2, 0.0f, 0.0f, 0.0f, e_2, 0.0f, 0.0f, 0.0f}, //THR
		{gp, 0.0f, 0.0f, 0.0f, 0.0f, lp, 0.0f, 0.0f, e_3, 0.0f, e_3, e_3, 0.0f, 0.0f, 0.0f}, //RUD
		{1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, //INVALID_1
		{gp, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, lp, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, e_1, 0.0f}, //ROLL_PITCH
		{gp, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, lp, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, e_1}, //ROLL_THRUST
		{gp, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, lp, 0.0f, 0.0f, 0.0f, e_2, e_2}, //ROLL_YAW
		{1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, //INVALID_2
		{gp, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, lp, 0.0f, e_1, 0.0f}, //PITCH_YAW
		{gp, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, lp, 0.0f, e_1}, //THUST_YAW
		{gp+e_1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, lp, 0.0f}, //MASSIVE_1
		{gp+e_1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, lp} //MASSIVE_2
	};

	memcpy(ttable, &temp_table, NUM_STATES * NUM_STATES * sizeof(float));

	PX4_INFO("Built transition table; groundstate prob: %f, loopback prob: %f", (double) gp, (double) lp);
}


void FWFailureAutomator::transition_state()
{
	float state_prob_dist[NUM_STATES]={0};
	for(uint8_t i = 0; i<NUM_STATES; i++) state_prob_dist[i] = ttable[fstate][i];

	fstate = static_cast<sys_failure_state_t>(_sample_from_dist(&state_prob_dist[0], NUM_STATES));
	fstate_action = STATE_ACTION[fstate];

	//PX4_INFO("state transitioned to state %d", fstate);
}


void FWFailureAutomator::run()
{
	// inits state and builds ttable
	parameters_update(true);

	vehicle_command_s switch_inst_cmd{};
	_make_switch_topic_inst_cmd(1, &switch_inst_cmd);
	_switch_topic_instance_cmd_pub.publish(switch_inst_cmd);
	PX4_INFO("switch topic instance to 1 cmd sent");
	//wait a bit command to be recieved by mixer process
	px4_usleep(800); // note: must be less than 1000us, the length of the simulator poll, or you'll get nasty errors

	int _actuator_controls_sub = orb_subscribe(ORB_ID(actuator_controls_0));
	px4_pollfd_struct_t fds[1];
	fds[0].fd = _actuator_controls_sub;
	fds[0].events = POLLIN;

	while (!should_exit()) {
		int pret = px4_poll(fds, 1, 50);

		if (pret == 0) {
			//50ms timeout on actuator controls poll, controller could be down
			continue;

		} else if (pret < 0) {
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000); //not much else we can do
			continue;

		} else if (fds[0].revents & POLLIN) { //valid poll

			// declared in header
			// actuator_controls_s input_vec;
			// actuator_controls_s output_vec;
			memset(input_vec.control, 0, sizeof(float) * 8);
			orb_copy(ORB_ID(actuator_controls_0), _actuator_controls_sub, &input_vec);
			// by default output is same as input
			memcpy(output_vec.control, input_vec.control, sizeof(float) * 8);

			// perform failure operations for each actuator
			for (uint8_t act_idx = AIL_L; act_idx < NUM_ACTUATORS; act_idx++) {
				// if the action vec says the actuator isnt effected, skip it
				if (!(fstate_action & (1 << act_idx))) { continue; }

				switch (fmode_param_vec[act_idx]) {
				case LINK_F: {
						output_vec.control[act_idx] = 0; //TODO: random stuck value instead of 0
						break;
					}

				case DELAY_F: {
						break; //WRITEME: implement delay
					}

				case PERTURB_F: {
						float gerr = _get_gausserror(_param_perturb_error_mean.get(), _param_perturb_error_sdev.get());
						output_vec.control[act_idx] = constrain(input_vec.control[act_idx] + gerr, -1.0f, 1.0f);
						break;
					}

				case CHAOTIC_F: {
						float gerr = _get_gausserror(_param_chaos_error_mean.get(), _param_chaos_error_sdev.get());
						output_vec.control[act_idx] = constrain(gerr, -1.0f, 1.0f);
						break;
					}

				default: {
						PX4_ERR("invalid failure mode provided, memory error likely");
						break;
					}
				}
			}

			output_vec.timestamp = hrt_absolute_time();
			output_vec.timestamp_sample = input_vec.timestamp;
			_actuator_controls_0_mpub.publish(output_vec);

		}
		// transition on interval
		if(hrt_absolute_time() - prev_trans > trans_interval && enable_transitions){
			transition_state();
			prev_trans = hrt_absolute_time();
		}

		do_log();
		parameters_update(false);
	}

	PX4_WARN("FWFA should_exit detected");
	//FIXME: exiting triggers inescapable race condition, so don't do that
	vehicle_command_s switch_inst_cmd_2{};
	_make_switch_topic_inst_cmd(0, &switch_inst_cmd_2);
	_switch_topic_instance_cmd_pub.publish(switch_inst_cmd_2);

	PX4_WARN("switch topic instance to 0 cmd sent, will likely fail");
	px4_usleep(500);

	orb_unsubscribe(_actuator_controls_sub);
}

void FWFailureAutomator::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);
		// update parameters from storage
		updateParams();

		build_fmode_vec();

		if (enable_transitions && !_param_enable_statetrans.get()) {
			PX4_INFO("state transitions disabled by param");
			enable_transitions = _param_enable_statetrans.get();

		} else if (!enable_transitions && _param_enable_statetrans.get()) {
			PX4_INFO("state transitions enabled by param");
			enable_transitions = _param_enable_statetrans.get();
		}

		trans_interval = 1000000 / _param_trans_freq.get();
		if(!float_eq(lb_prob,_param_loopback_prob.get()) || !float_eq(gs_prob, _param_groundstate_prob.get())){
			lb_prob = _param_loopback_prob.get();
			gs_prob = _param_groundstate_prob.get();

			build_ttable(lb_prob, gs_prob);
			PX4_INFO("new transtion table built from params");
		}

		// only listen to new enable states via param if we won't be transitioning away anyways
		if (!enable_transitions) {
			uint16_t paramstate = _param_init_state.get();
			fstate = static_cast<sys_failure_state_t>(paramstate);
			PX4_INFO("reverted state to initstate defined by parameters");
		}
	}
}

void FWFailureAutomator::do_log()
{
	//WRITEME: logger
}

FWFailureAutomator *FWFailureAutomator::instantiate(int argc, char *argv[])
{
	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "h", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	FWFailureAutomator *instance = new FWFailureAutomator();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int FWFailureAutomator::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("fw_failure_automator",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      2048,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int FWFailureAutomator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int FWFailureAutomator::print_status()
{	PX4_INFO("actuator failure state => %d", fstate);
	PX4_INFO("actuator action space => [%s: %d, %s: %d,%s: %d, %s: %d, %s: %d]",
		 act_names[0], (bool)(fstate_action & 1),
		 act_names[1], (bool)(fstate_action & 2),
		 act_names[2], (bool)(fstate_action & 4),
		 act_names[3], (bool)(fstate_action & 8),
		 act_names[4], (bool)(fstate_action & 16)
		);
	PX4_INFO("actuator failure modes => [%s: %s, %s: %s,%s: %s, %s: %s, %s: %s]",
		 act_names[0], fmode_names[fmode_param_vec[0]],
		 act_names[1], fmode_names[fmode_param_vec[1]],
		 act_names[2], fmode_names[fmode_param_vec[2]],
		 act_names[3], fmode_names[fmode_param_vec[3]],
		 act_names[4], fmode_names[fmode_param_vec[4]]
		);

	return 0;
}

int FWFailureAutomator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_failure_automator models random failures across actuators as a stocastic dynamical system.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_failure_automator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int fw_failure_automator_main(int argc, char *argv[])
{
	return FWFailureAutomator::main(argc, argv);
}

