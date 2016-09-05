Vizualization of xy, tx and ty projection of a trajectory. A trajectory is selected by clicing on the corresponding color dot in frame.
Frames can be traversed forward and bacward by pressing 'f' and 'b' buttoms respectively. The projections are shown in separate windows.
Press 'r' to remove all windows exept of the main window.

Requirements: Opencv 2.4, g++ with C++11 support, make and pkg-config

Usage ./trajectory_vizualization <path_to_trajectories> <path_to_partition> <path_to_frames> <graph_scale>
	 <scale_factor> should be in [1...100]

- <path_to_trajectories> <path_to_partition> are dat files with following structure:
	<video_length>
	<amount_of_trajectories>
	<trajectory_label> <n - length_of_trajectory>
	<x_0> <y_0> <t_0>
	<x_1> <y_1> <t_1>
	...
	<x_n-1> <y_n-1> <t_n-1>
	<trajectory_label> <n - length_of_trajectory>
	<x_0> <y_0> <t_0>
	<x_1> <y_1> <t_1>
	...
	<x_n-1> <y_n-1> <t_n-1>
	...
	<trajectory_label> <n - length_of_trajectory>
	<x_0> <y_0> <t_0>
	<x_1> <y_1> <t_1>
	...
	<x_n-1> <y_n-1> <t_n-1>
	Note: <trajectory_label> is a random int number. It is keeped only for compatability

- <path_to_frames> is a bmf file:
	<m - amount_of_frames> 1
	<path_to_frame_0>
	<path_to_frame_1>
	...
	<path_to_frame_m-1>

- <graph_scale> is used for enlarging sides of plots. Maximal length of the larger side of a plot is 1000 pixels.
	It preserves aspect ratio 1:1
