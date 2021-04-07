#include <Eigen/Eigen>


class DigitalFilter {
	public:
		DigitalFilter(int order, double * xGains, double init_value)
		{
			len = order + 1;
			x_gain.resize(len);
			x.resize(len);
			y.resize(len);

			for(int i=0;i<len;i++)
			{
					x_gain(i) = xGains[i];
					x(i) = init_value;
					y(i) = init_value;
			}
			// ROS_INFO_STREAM("Filter initialized with:");
			// ROS_INFO_STREAM("gains\n" << x_gain);
			// ROS_INFO_STREAM("x\n" << x);
			// ROS_INFO_STREAM("y\n" << y);
		}

		DigitalFilter(int order, double init_value)
		{
			len = order + 1;
			x_gain.resize(len);
			x.resize(len);
			y.resize(len);

			for(int i=0;i<len;i++)
			{
				x_gain(i) = 1.0/(double)len;
				x(i) = init_value;
				y(i) = init_value;
			}
			// ROS_INFO_STREAM("Simple Filter initialized with:");
			// ROS_INFO_STREAM("gains\n" << x_gain);
			// ROS_INFO_STREAM("x\n" << x);
			// ROS_INFO_STREAM("y\n" << y);
		}

		double filter(double newVal)
		{
			for(int i=(len-1); i>0; i--)//will run len-1 times because the last value will set after
			{
				x(i) = x(i-1);
				y(i) = y(i-1);
			}
			x(0) = newVal;
			y(0) = x * x_gain;
			return y(0);
		}

	private:
		int len;
		Eigen::VectorXd x_gain;
		Eigen::RowVectorXd x, y;
};

