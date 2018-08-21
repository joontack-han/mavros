#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/EstimatorStatus.h>

namespace mavplugin {
/**
 * @brief OdarWrench plugin.
 */
class EstimatorStatusPlugin : public MavRosPlugin {
public:
	EstimatorStatusPlugin() :
        	nh("~odar"), uas(nullptr)
	{ }

    /**
     * Plugin initializer. Constructor should not do this.
     */
	void initialize(UAS &uas_)
	{
 		uas = &uas_;
		estimator_status_pub = nh.advertise<mavros_msgs::EstimatorStatus>("est_status", 10);
	}

	const message_map get_rx_handlers() {
		return {
                	MESSAGE_HANDLER(MAVLINK_MSG_ID_ESTIMATOR_STATUS, &EstimatorStatusPlugin::handle_estimator_status),
	};
	}

private:
	ros::NodeHandle nh;
	UAS *uas;


	ros::Publisher estimator_status_pub;

	void handle_estimator_status(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_estimator_status_t estimator_status;
		mavlink_msg_estimator_status_decode(msg, &estimator_status);

		auto ros_msg = boost::make_shared<mavros_msgs::EstimatorStatus>();

		uint32_t sec, ns;
		sec = estimator_status.time_usec/1000000;
		ns = (estimator_status.time_usec - sec*1000000)*1000;
		ros_msg->header.stamp = ros::Time(sec, ns);

		//ros_msg->covariances[i] = estimator_status.covariances[i];
		ros_msg->cov_qw = estimator_status.cov_qw;
		ros_msg->cov_qx = estimator_status.cov_qx;
		ros_msg->cov_qy = estimator_status.cov_qy;
		ros_msg->cov_qz = estimator_status.cov_qz;
		ros_msg->cov_px = estimator_status.cov_px;
		ros_msg->cov_py = estimator_status.cov_py;
		ros_msg->cov_pz = estimator_status.cov_pz;

		
		estimator_status_pub.publish(ros_msg);
	}

};
};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::EstimatorStatusPlugin, mavplugin::MavRosPlugin)
