#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_msgs/EstimatorCovariance.h>

namespace mavplugin {
/**
 * @brief OdarWrench plugin.
 */
class EstimatorCovariancePlugin : public MavRosPlugin {
public:
	EstimatorCovariancePlugin() :
        	nh("~odar"), uas(nullptr)
	{ }

    /**
     * Plugin initializer. Constructor should not do this.
     */
	void initialize(UAS &uas_)
	{
 		uas = &uas_;
		estimator_covariance_pub = nh.advertise<mavros_msgs::EstimatorCovariance>("est_covariance", 10);
	}

	const message_map get_rx_handlers() {
		return {
                	MESSAGE_HANDLER(MAVLINK_MSG_ID_ESTIMATOR_COVARIANCE, &EstimatorCovariancePlugin::handle_estimator_covariance),
	};
	}

private:
	ros::NodeHandle nh;
	UAS *uas;


	ros::Publisher estimator_covariance_pub;

	void handle_estimator_covariance(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_estimator_covariance_t estimator_covariance;
		mavlink_msg_estimator_covariance_decode(msg, &estimator_covariance);

		auto ros_msg = boost::make_shared<mavros_msgs::EstimatorCovariance>();

		uint32_t sec, ns;
		sec = estimator_covariance.time_usec/1000000;
		ns = (estimator_covariance.time_usec - sec*1000000)*1000;
		ros_msg->header.stamp = ros::Time(sec, ns);

		//ros_msg->covariances[i] = estimator_status.covariances[i];

		for (int i=0; i<45; i++) {
			ros_msg->covariance[i] = estimator_covariance.cov[i];
		}

		
		estimator_covariance_pub.publish(ros_msg);
	}

};
};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::EstimatorCovariancePlugin, mavplugin::MavRosPlugin)
