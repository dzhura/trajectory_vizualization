Vizualization of xy, tx and ty projection of a trajectory. A trajectory is selected by clicing on the corresponding color dot in frame.
Frames can be traversed forward and bacward by pressing 'f' and 'b' buttoms respectively. The projections are shown in separate windows.
Press 'r' to remove all windows exept of the main window.

Requirements: OpenCV 2.4, g++ with C++11 support, GNU make, pkg-config and gnuplot

gnuplot_i is developed by N. Devillard: http://ndevilla.free.fr/gnuplot/

Usage ./trajectory_vizualization <path_to_trajectories> <path_to_partition> <path_to_frames>

- <path_to_trajectories> <path_to_partition> are dat files with following structure:
	<video_length> <amount_of_trajectories>
	<trajectory_label> <n - length_of_trajectory>
	<x_0> <y_0> <t_0>
	<x_1> <y_1> <t_1>
	...
	<x_n-1> <y_n-1> <t_n-1>
	<trajectory_label> <m - length_of_trajectory>
	<x_0> <y_0> <t_0>
	<x_1> <y_1> <t_1>
	...
	<x_m-1> <y_m-1> <t_m-1>
	...
	<trajectory_label> <k - length_of_trajectory>
	<x_0> <y_0> <t_0>
	<x_1> <y_1> <t_1>
	...
	<x_k-1> <y_k-1> <t_k-1>
	Note: trajectory_label is a random int number. It is keeped only for compatability
		x_* and y_* are position of a trajectory at frame t_*

- <path_to_partition> is a dat file with following structure:
	<video_length> <amount_of_trajectories>
	<n - length_of_trajectory>
	<i_0>
	<i_1>
	...
	<i_n-1>
	<m - length_of_trajectory>
	<i_0>
	<i_1>
	...
	<i_m-1>
	...
	<k - length_of_trajectory>
	<i_0>
	<i_1>
	...
	<i_k-1>
	Note: i_* is a index of point of the corresponding trajectory, where the point is boundary btw two motions

- <path_to_frames> is a bmf file:
	<m - amount_of_frames> 1
	<path_to_frame_0>
	<path_to_frame_1>
	...
	<path_to_frame_m-1>
