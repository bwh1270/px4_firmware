#ifndef HOVER_THRUST_ESTIMATE_HPP
#define HOVER_THRUST_ESTIMATE_HPP

#include <uORB/topics/hover_thrust_estimate.h>

class MavlinkStreamHoverThrustEstimate : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamHoverThrustEstimate(mavlink); }

	static constexpr const char *get_name_static() { return "HOVER_THRUST_ESTIMATE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _hover_thr_esti_sub.advertised() ? (MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamHoverThrustEstimate(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _hover_thr_esti_sub{ORB_ID(hover_thrust_estimate)};

	bool send() override
	{
		// we're sending the GPS home periodically to ensure the
		// the GCS does pick it up at one point
		hover_thrust_estimate_s thr_h;
		if (_hover_thr_esti_sub.copy(&thr_h)) {

			mavlink_hover_thrust_estimate_t msg{};
			msg.time_usec = thr_h.timestamp;
			msg.thr_h_hat = thr_h.hover_thrust;
			msg.thr_h_var = thr_h.hover_thrust_var;
			msg.valid = thr_h.valid;

			mavlink_msg_hover_thrust_estimate_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		// if (_hover_thr_esti_sub.advertised() && _hover_thr_esti_sub.copy(&thr_h)) {
		// 	if (home.valid_hpos) {
		// 		mavlink_home_position_t msg{};

		// 		msg.latitude  = home.lat * 1e7;
		// 		msg.longitude = home.lon * 1e7;
		// 		msg.altitude  = home.alt * 1e3f;

		// 		msg.x = home.x;
		// 		msg.y = home.y;
		// 		msg.z = home.z;

		// 		matrix::Quatf q(matrix::Eulerf(0.f, 0.f, home.yaw));
		// 		q.copyTo(msg.q);

		// 		msg.approach_x = 0.f;
		// 		msg.approach_y = 0.f;
		// 		msg.approach_z = 0.f;

		// 		msg.time_usec = home.timestamp;

		// 		mavlink_msg_hover_thrust_estimate_struct(_mavlink->get_channel(), &msg);

		// 		return true;
		// 	}
		// }

		return false;
	}
};

#endif // HOVER_THRUST_ESTIMATE
