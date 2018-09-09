#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <mavros_msgs/OdarSCKF.h>

namespace mavplugin {

	class OdarSCKFPlugin : public MavRosPlugin {
		public:
			OdarSCKFPlugin() :
				nh("~odar"),
				uas(nullptr)
		{ };

			void initialize(UAS &uas_)
			{
				uas = &uas_;
				odar_sckf_sub = nh.subscribe("sckf", 10, &OdarSCKFPlugin::odar_sckf_cb, this);
			};

			const message_map get_rx_handlers() {
				return {/* RX disabled */ };
			}

		private:
			ros::NodeHandle nh;
			UAS *uas;
			ros::Subscriber odar_sckf_sub;

			void odar_sckf_cb(const mavros_msgs::OdarSCKF::ConstPtr &req)
			{

				mavlink_message_t msg;
				mavlink_msg_odar_sckf_pack_chan(UAS_PACK_CHAN(uas), &msg, 
					req->header.stamp.toNSec() / 1000,
					req->quat[0], req->quat[1], req->quat[2], req->quat[3],
					req->vel[0], req->vel[1], req->vel[2], 
					req->pos[0], req->pos[1], req->pos[2]);
				UAS_FCU(uas)->send_message(&msg);

		

			}
	};
};

PLUGINLIB_EXPORT_CLASS(mavplugin::OdarSCKFPlugin, mavplugin::MavRosPlugin)
