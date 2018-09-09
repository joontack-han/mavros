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
					req->link_idx,
					req->constraint_error[0], req->constraint_error[1], req->constraint_error[2], req->constraint_error[3], req->constraint_error[4], req->constraint_error[5],
					req->error_covariance_1[0], req->error_covariance_1[1], req->error_covariance_1[2], req->error_covariance_1[3], req->error_covariance_1[4], req->error_covariance_1[5],
					req->error_covariance_2[0], req->error_covariance_2[1], req->error_covariance_2[2], req->error_covariance_2[3], req->error_covariance_2[4], req->error_covariance_2[5]);
				UAS_FCU(uas)->send_message(&msg);

		

			}
	};
};

PLUGINLIB_EXPORT_CLASS(mavplugin::OdarSCKFPlugin, mavplugin::MavRosPlugin)
