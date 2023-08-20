/**
 * Switch for enabling/disabling dynamic fail state transitions
 *
 * @boolean
 * @group FW AutoFail Params
 */
PARAM_DEFINE_INT32(FW_AF_DYNSYS_SW, 1);

/**
 * Frequency of markov state transitions in hz
 *
 * @group FW AutoFail Params
*/
PARAM_DEFINE_FLOAT(FW_AF_TRANS_FREQ, 1);

/**
 * Probability of remaining in the same state in a given transition
 * @min 0
 * @max 0.5
 * @group FW AutoFail Params
*/
PARAM_DEFINE_FLOAT(FW_AF_LB_PROB, 0.3);

/**
 * Probability of returning to the ground state in a given transition
 * @min 0
 * @max 0.5
 * @group FW AutoFail Params
*/
PARAM_DEFINE_FLOAT(FW_AF_GS_PROB, 0.4);

/**
 * Initial state of failure automator
 * @value 0 NO_FAIL
 * @value 1 LAIL_GUM
 * @value 2 RAIL_GUM
 * @value 3 ELE_GUM
 * @value 4 THR_GUM
 * @value 5 RUD_GUM
 * @value 7 ROLL_PITCH
 * @value 8 ROLL_THRUST
 * @value 9 ROLL_YAW
 * @value 11 PITCH_YAW
 * @value 12 THRUST_YAW
 * @value 13 MASSIVE_1
 * @value 14 MASSIVE_2
 * @group FW AutoFail Params
 */
PARAM_DEFINE_INT32(FW_AF_ISTATE, 0); //defaults to all failures enabled 0 -> no fail


/**
 * Failure Mode for Left Aileron
 * @value 0 NOFAIL_F
 * @value 1 LINK_F 	// hold current actuator value as if the controller lost the link
 * @value 2 DELAY_F  	// drastically delay the signal the controller sends as if there was a timing error
 * @value 3 PERTURB_F 	// reemit true ac values with probabilistic offset
 * @value 4 CHAOTIC_F	// send servo into chaotic motion as if it entered adversarial conditions e.g. servo was destroyed
 * @group FW AutoFail Params
 */
PARAM_DEFINE_INT32(FW_AF_LAIL_FMODE, 0);

/**
 * Failure Mode for Right Aileron
 * @value 0 NOFAIL_F
 * @value 1 LINK_F 	// hold current actuator value as if the controller lost the link
 * @value 2 DELAY_F  	// drastically delay the signal the controller sends as if there was a timing error
 * @value 3 PERTURB_F 	// reemit true ac values with probabilistic offset
 * @value 4 CHAOTIC_F	// send servo into chaotic motion as if it entered adversarial conditions e.g. servo was destroyed
 * @group FW AutoFail Params
 */
PARAM_DEFINE_INT32(FW_AF_RAIL_FMODE, 0);

/**
 * Failure Mode for Elevator
 * @value 0 NOFAIL_F
 * @value 1 LINK_F 	// hold current actuator value as if the controller lost the link
 * @value 2 DELAY_F  	// drastically delay the signal the controller sends as if there was a timing error
 * @value 3 PERTURB_F 	// reemit true ac values with probabilistic offset
 * @value 4 CHAOTIC_F	// send servo into chaotic motion as if it entered adversarial conditions e.g. servo was destroyed
 * @group FW AutoFail Params
 */
PARAM_DEFINE_INT32(FW_AF_ELE_FMODE, 0);

/**
 * Failure Mode for Throttle
 * @value 0 NOFAIL_F
 * @value 1 LINK_F 	// hold current actuator value as if the controller lost the link
 * @value 2 DELAY_F  	// drastically delay the signal the controller sends as if there was a timing error
 * @value 3 PERTURB_F 	// reemit true ac values with probabilistic offset
 * @value 4 CHAOTIC_F	// send servo into chaotic motion as if it entered adversarial conditions e.g. servo was destroyed
 * @group FW AutoFail Params
 */
PARAM_DEFINE_INT32(FW_AF_THR_FMODE, 0);

/**
 * Failure Mode for Rudder
 * @value 0 NOFAIL_F
 * @value 1 LINK_F 	// hold current actuator value as if the controller lost the link
 * @value 2 DELAY_F  	// drastically delay the signal the controller sends as if there was a timing error
 * @value 3 PERTURB_F 	// reemit true ac values with probabilistic offset
 * @value 4 CHAOTIC_F	// send servo into chaotic motion as if it entered adversarial conditions e.g. servo was destroyed
 * @group FW AutoFail Params
 */
PARAM_DEFINE_INT32(FW_AF_RUD_FMODE, 0);

/**
 * Mean of perturbation error
 * @min -0.25
 * @max 0.25
 * @group FW AutoFail Params
*/
PARAM_DEFINE_FLOAT(FW_AF_PTURB_M, 0.0f);


/**
 * Mean of chaotic error
 * @min -0.95
 * @max 0.95
 * @group FW AutoFail Params
*/
PARAM_DEFINE_FLOAT(FW_AF_CHAOS_M, 0.0f);

/**
 * Standard deviation of perturbation error
 * @min 0.001
 * @max 0.2
 * @group FW AutoFail Params
*/
PARAM_DEFINE_FLOAT(FW_AF_PTURB_SD, 0.02f);

/**
 * Standard deviation of chaotic error
 * @min 0.001
 * @max 0.2
 * @group FW AutoFail Params
*/
PARAM_DEFINE_FLOAT(FW_AF_CHAOS_SD, 0.02f);
