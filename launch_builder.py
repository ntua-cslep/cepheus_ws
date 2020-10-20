import argparse


def build_launch(thruster_force='0.6',
				max_motor_current='2.00',
				rw_max_speed='400',
				rw_total_inertia='0.00197265',
				rw_max_torque='0.02',
				rw_max_power='30',
				loop_rate='200.0',
				left_shoulder_limit_pos='3.1548',
				left_elbow_limit_pos='1.658',
				right_shoulder_limit_pos='-2.453',
				right_elbow_limit_pos='-1.6231',
				use_with_chase_planner=True
	):

	with open('test.launch', 'a') as file:
		file.write('<launch>\n')
		file.write('\t<param name="robot_description" command="cat $(find cepheus_robot)/model/cepheus.urdf" />\n\n')
		file.write('\t<rosparam command="load" file="$(find cepheus_robot)/config/controllers.yaml"/>\n\n')
		file.write('\t<node name="cepheus_interface_node" pkg="cepheus_robot" type="cepheus_interface_node" respawn="false" output="screen" required="true" >\n')
		file.write('\t\t<param name=\"thruster_force\" value="%s"/>\t\n' % thruster_force)
		file.write('\t\t<param name=\"max_motor_current\" value="%s"/>\t\n' % max_motor_current)
		file.write('\t\t<param name=\"rw_max_speed\" value="%s"/>\t\n' % rw_max_speed)
		file.write('\t\t<param name=\"rw_total_inertia\" value="%s"/>\t\n' % rw_total_inertia)
		file.write('\t\t<param name=\"rw_max_torque\" value="%s"/>\t\n' % rw_max_torque)
		file.write('\t\t<param name=\"rw_max_power\" value="%s"/>\t\n' % rw_max_power)
		file.write('\t\t<param name=\"loop_rate\" value="%s"/>\t\n' % loop_rate)
		file.write('\t\t<param name=\"left_shoulder_limit_pos\" value="%s"/>\t\n' % left_shoulder_limit_pos)
		file.write('\t\t<param name=\"left_elbow_limit_pos\" value="%s"/>\t\n' % left_elbow_limit_pos)
		file.write('\t\t<param name=\"right_shoulder_limit_pos\" value="%s"/>\t\n' % right_shoulder_limit_pos)
		file.write('\t\t<param name=\"right_elbow_limit_pos\" value="%s"/>\t\n' % right_elbow_limit_pos)
		file.write('\t\t<param name=\"use_with_chase_planner\" value="%s"/>\t\n' % use_with_chase_planner)
		file.write('\t</node>\n\n')
		file.write('\t<node name="cepheus_ctrl" pkg="cepheus_robot" type="cepheus_ctrl" respawn="false" output="screen" required="true">\n')
		file.write('\t\t<param name="controllers_to_spawn" value="joint_state_publisher reaction_wheel_effort_controller right_shoulder_position_controller right_elbow_position_controller"/>\n')
		file.write('\t</node>\n\n')
		file.write('\t<node name="cepheus_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" output="screen" required="true">\n')
		file.write('\t\t<param name="publish_frequency" value="200"/>\n')
		file.write('\t</node>\n\n')
		file.write('</launch>\n')
		file.close()


if __name__ == '__main__':
	# parser = argparse.ArgumentParser(description='Build cepheus launch file.')
	# args = parser.parse_args()
	build_launch()

