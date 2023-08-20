/****************************************************************************
 *
 *   Copyright (c) 2022 MIT ESSG. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permittedprovided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file FWFailureAutomator.hpp
 *
 * Simulate actuator failures via a stocastic dynamical system.
 *
 * @author Thelonious Cooper <theloni@mit.edu>
 */
//TODO: make this its own git repo and import as an out-of-tree module
/**
 * Notes on module structure:
 * the pxio driver in src/drivers subscribes to the actuator_controls_0 uorb msg
 * We modify this to make it subscribe_multi to a specific instance of actuator_controls_0
 * then we generate our own faulty instance of the actuator_controls_0 dds buffer
 * we then send a custom uorb message to tell the pxio driver to switch its subscribe_multi instance
 */
#include <matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

using namespace time_literals;

extern "C" __EXPORT int fw_failure_automator_main(int argc, char *argv[]);

static constexpr uint8_t NUM_ACTUATORS = 5;
typedef enum fw_actuator : uint8_t {
	AIL_L,
	AIL_R,
	ELE,
	THR,
	RUD
} fw_actuator_t;

typedef enum failure_mode : uint8_t {
	LINK_F, 	// hold current actuator value as if the controller lost the link
	DELAY_F, 	// drastically delay the signal the controller sends as if there was a timing error
	PERTURB_F, 	// reemit true ac values with probabilistic offset
	CHAOTIC_F 	// send servo into chaotic motion as if it entered adversarial conditions e.g. servo was destroyed
} failure_mode_t;

static constexpr uint8_t NUM_STATES = 15;
typedef enum sys_failure_state : uint8_t {
	NO_FAIL,
	LAIL_GUM,
	RAIL_GUM,
	ELE_GUM,
	THR_GUM,
	RUD_GUM,
	INVALID_1,
	ROLL_PITCH,
	ROLL_THRUST,
	ROLL_YAW,
	INVALID_2,
	PITCH_YAW,
	THRUST_YAW,
	MASSIVE_1,
	MASSIVE_2
} sys_failure_state_t;

uint8_t STATE_ACTION[NUM_STATES] = {
	0b00000, //NO_FAIL
	0b00001, //LAIL_GUM
	0b00010, //RAIL_GUM
	0b00100, //ELE_GUM
	0b01000, //THR_GUM
	0b10000, //RUD_GUM
	0b00011, //INVALID_1
	0b00101, //ROLL_PITCH
	0b01001, //ROLL_THRUST
	0b10001, //ROLL_YAW
	0b01100, //INVALID_2
	0b10100, //PITCH_YAW
	0b11000, //THRUST_YAW
	0b10101, //MASSIVE_1
	0b11001  //MASSIVE_2
}; //actions in byte form, i.e. bitshift to recover


// namespace avoids name collisions on common var names like markov_data
/// misc helpers for failure automation
namespace fwfa_namespace
{
/**
 * @brief Helper to fill in the command, timestamps, and valid params for a
 * vehical command
 *
 * @param inst
 * @param cmd
 */
inline void _make_switch_topic_inst_cmd(uint8_t inst, vehicle_command_s *cmd);

/**
 * @brief Helper to generate an error to apply to an actuator
 *
 * @param mean
 * @param sdev
 * @return float
 */
float _get_gausserror(float mean, float sdev);

/**
 * @brief Samples from a given probability distribution and returns an index
 *
 * @param dist
 * @param dist_len
 * @return uint
 */
uint8_t _sample_from_dist(float* dist, size_t dist_len);

const char fmode_names[4][8] = {"___LINK", "__DELAY", "PERTURB", "CHAOTIC"};

const char act_names[5][6] = {"AIL_L", "AIL_R", "__ELE", "__THR", "__RUD"};

}

class FWFailureAutomator: public ModuleBase<FWFailureAutomator>, public ModuleParams
{

public:
	FWFailureAutomator();

	~FWFailureAutomator() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static FWFailureAutomator *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void run() override;

private:

	//WRITEME: publish uorb topic of state and log it
	sys_failure_state_t fstate = NO_FAIL;
	/**
	 * @brief a boolean vector(byte) of which actuators are being fuzzed
	 * This actoni vector will change according to the state
	 */
	uint8_t fstate_action = 0;

	void build_fmode_vec();
	failure_mode_t fmode_param_vec [NUM_ACTUATORS]; /// vector of how to perturb each actuator

	//TODO: implement delay with ring buffer of input_vecs
	struct actuator_controls_s input_vec; /// actuator values sent from controller
	struct actuator_controls_s output_vec; /// fuzzed values to be re-emitted

	// for telling the mixer driver to listen to the correct topic instance
	uORB::Publication<vehicle_command_s> _switch_topic_instance_cmd_pub{ORB_ID(vehicle_command)};

	// for re-pubbing the fuzzed actuator controls
	uORB::PublicationMulti<actuator_controls_s>	_actuator_controls_0_mpub{ORB_ID(actuator_controls_0)};


	/**
	* @brief builds the transition table for the markov process based with even probabilities along
	* pre-defined edges and a provided probability of returning to the same state or going back to ground.
	*
	* @param loopback_prob [0, 0.5)
	* @param groundstate_prob [0, 0.5)
	* @param table reference to table
	*/
	void build_ttable(float loopback_prob, float groundstate_prob);
	float lb_prob = -1; // reset by param later
	float gs_prob = -1;
	float ttable[NUM_STATES][NUM_STATES] = {};
	unsigned long trans_interval;
	long prev_trans = 0;
	bool enable_transitions = true;
	/**
	* @brief inverse transform sampling CDF method to generate new state from transition table
	* collapse state to action vector
	*/
	void transition_state(); //TODO: implement playback mode

	/**
	 * @brief log fuzzer state,
	 * along with diff between expected and fuzzed ac vals
	 * so it can be cross-referenced against the recorded attitude
	 */
	//WRITEME: logging
	void do_log();

	// Subscription to Parameters only checks once per sec
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	/**
	 * @brief retrieve parameters from parameter_update sub, if they have changed or we force it
	 * @param force
	 */
	void parameters_update(bool force);

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FW_AF_DYNSYS_SW>) _param_enable_statetrans,
		(ParamFloat<px4::params::FW_AF_TRANS_FREQ>) _param_trans_freq,
		(ParamFloat<px4::params::FW_AF_LB_PROB>) _param_loopback_prob,
		(ParamFloat<px4::params::FW_AF_GS_PROB>) _param_groundstate_prob,
		(ParamInt<px4::params::FW_AF_ISTATE>) _param_init_state,

		(ParamInt<px4::params::FW_AF_LAIL_FMODE>) _param_lail_fmode,
		(ParamInt<px4::params::FW_AF_RAIL_FMODE>) _param_rail_fmode,
		(ParamInt<px4::params::FW_AF_ELE_FMODE>) _param_ele_fmode,
		(ParamInt<px4::params::FW_AF_THR_FMODE>) _param_thr_fmode,
		(ParamInt<px4::params::FW_AF_RUD_FMODE>) _param_rud_fmode,

		(ParamFloat<px4::params::FW_AF_PTURB_SD>) _param_perturb_error_sdev,
		(ParamFloat<px4::params::FW_AF_PTURB_M>) _param_perturb_error_mean,
		(ParamFloat<px4::params::FW_AF_CHAOS_SD>) _param_chaos_error_sdev,
		(ParamFloat<px4::params::FW_AF_CHAOS_M>) _param_chaos_error_mean
	)
};
