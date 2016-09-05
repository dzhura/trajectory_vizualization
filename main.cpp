// (c) Maxim Dzhura Sep 2016
//
// Reads trajectories and their partition from two .dat files and corresponding frames from a .bmf file
// Show trajectories in the frames and provides x,y projections for each trajectory as well as acceleration
#include <vector>
#include <cassert>
#include <algorithm> // min_element max_element fill_n

#include <fstream>
#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

const unsigned int max_win_size = 1000;

struct trajectory_t
{
        trajectory_t() { } 
	trajectory_t(size_t size):
       		_x(size), _y(size), _t(size) { } 

	void resize(size_t size)
       	{ 
		_x.resize(size);
	       	_y.resize(size);
	       	_t.resize(size); 
	} 
	
	size_t size() const 
	{
		return _t.size();
	} 
	
	std::vector<float> _x, _y; 
	std::vector<int> _t; 
}; //trajectory_t

typedef std::vector<unsigned int> partition_t;

// maps a voxel (x,y,t) to an index of trajectory, which it belongs to
class voxel_map_t
{
	public:
	voxel_map_t():
       		_widht(0), _height(0), _video_length(0), _data(0) { }
	voxel_map_t(int width, int height, int video_length):
		_widht(width), _height(height), _video_length(video_length),
	       	_data(new int[_widht*_height*_video_length])
	{
		std::fill_n(_data, _widht*_height*_video_length, -1);
	}

	~voxel_map_t()
	{
		delete [] _data;
	}

	void resize(int width, int height, int video_length)
	{
		if( _data!=0) {
			delete [] _data;
		}

		_widht = width; _height = height; _video_length = video_length;
		_data = new int[_widht*_height*_video_length];
		std::fill_n(_data, _widht*_height*_video_length, -1);
	}

	int & operator() (int x, int y, int t) const
	{
		assert( -1 < x && x < _widht );
		assert( -1 < y && y < _height );
		assert( -1 < t && t < _video_length );

		return _data[_shift(x, y, t)];
	}


	private:
	int _widht;
	int _height;
	int _video_length;

	int * _data;

	private:

	size_t _shift(int x, int y, int t) const {
		return x + _widht*(y + t*_height);
	}

	voxel_map_t(const voxel_map_t &);
	voxel_map_t(voxel_map_t &);
	voxel_map_t & operator=(const voxel_map_t &);
	
}; // voxel_map_t

// A struct for passing arguments to SetMouseCallback
struct mouse_callback_input_t
{
	const unsigned int & _current_frame_number;
	const voxel_map_t & _trajectory_id;
	const std::vector<trajectory_t> & _trajectories;
	const std::vector<partition_t> & _partitions;
	float _scale;

	mouse_callback_input_t( const unsigned int & current_frame_number, const voxel_map_t & trajectory_id,
	       			const std::vector<trajectory_t> & trajectories,
			       	const std::vector<partition_t> & partitions, float scale):
			       	_current_frame_number(current_frame_number), _trajectory_id(trajectory_id),
			       	_trajectories(trajectories),
				_partitions(partitions), _scale(scale) { }
}; // mouse_callback_input_t

static void show_graphs( int event, int x, int y, int dummy, void * args);

void draw_xy_projection( const trajectory_t & tr, const partition_t & parts, cv::Mat & out, float scale);
void draw_tx_projection( const trajectory_t & tr, const partition_t & parts, cv::Mat & out, float scale);
void draw_ty_projection( const trajectory_t & tr, const partition_t & parts, cv::Mat & out, float scale);

void draw_2D_axies(cv::Mat & region, const cv::Size & border);

void draw_trajectories(const std::vector<trajectory_t> & trajectories, std::vector<cv::Mat> & video, voxel_map_t & pos_2_trajectory);

int main(int argc, char * argv[]) 
{
	if(argc!=1+4) {
		std::cout << "Usage: " << argv[0] << " <path_to_trajectories> <path_to_partition> <path_to_frames> <graph_scale>" << std::endl;
		std::cout << "\t <scale_factor> should be in [1...100]" << std::endl;
		return 1;
	}

	std::string path_to_trajectories(argv[1]);
	if( path_to_trajectories.compare(path_to_trajectories.find_last_of("."), std::string::npos, ".dat") != 0 ) {
		std::cout << path_to_trajectories << " must be a .dat file" << std::endl;
		return 1;
	}

	std::string path_to_partition(argv[2]);
	if( path_to_partition.compare(path_to_partition.find_last_of("."), std::string::npos, ".dat") != 0 ) {
		std::cout << path_to_partition << " must be a .dat file" << std::endl;
		return 1;
	}

	std::string path_to_frames(argv[3]);
	if( path_to_frames.compare(path_to_frames.find_last_of("."), std::string::npos, ".bmf") != 0 ) {
		std::cout << path_to_frames << " must be a .bmf file" << std::endl;
		return 1;
	}

	float graph_scale = atof(argv[4]);
	if( graph_scale < 1 || 100 < graph_scale ) {
		std::cout << "Graph scale should be in [1...100], but it is " << graph_scale << std::endl;
		return 1;
	}


	//// read input
	// trajectories
	std::ifstream in_trajectoires(path_to_trajectories);
	if( !in_trajectoires.is_open() ) {
		std::cout << "Canot " << path_to_trajectories << std::endl;
		return 1;
	}

	unsigned int video_length;
	unsigned int trajectory_amount;
	in_trajectoires >> video_length >> trajectory_amount;

	std::vector<trajectory_t> trajectories(trajectory_amount);
	for(size_t i=0; i<trajectories.size(); ++i) {
		int label;
		unsigned int trajectory_size;
		in_trajectoires >> label >> trajectory_size;

		trajectories[i].resize(trajectory_size);
		for(size_t j=0; j < trajectories[i].size(); ++j) {
			in_trajectoires >> trajectories[i]._x[j] >> trajectories[i]._y[j] >> trajectories[i]._t[j];
			trajectories[i]._x[j] = lroundf(trajectories[i]._x[j]);
			trajectories[i]._y[j] = lroundf(trajectories[i]._y[j]);
		}
	}
	in_trajectoires.close();

	// partition of trajectories
	std::ifstream in_partition(path_to_partition);
	if( !in_partition.is_open() ) {
		std::cout << "Canot " << path_to_partition << std::endl;
		return 1;
	}

	unsigned int video_length_2;
	unsigned int trajectory_amount_2;
	in_partition >> video_length_2 >> trajectory_amount_2;

	std::vector<partition_t> partitions(trajectory_amount_2);
	for(size_t i=0; i<partitions.size(); ++i) {
		unsigned int partition_amount;
		in_partition >> partition_amount;

		partitions[i].resize(partition_amount);
		for(size_t j=0; j < partitions[i].size(); ++j) {
			in_partition >> partitions[i][j];
		}
	}
	in_partition.close();

	// frames
	std::string root_dir = path_to_frames.substr(0, path_to_frames.find_last_of("/")+1);
	if(root_dir.compare(path_to_frames) == 0) { // if frames are in current folder
		root_dir = "./";
	}

	std::ifstream in_frames(path_to_frames);
	if( !in_frames.is_open() ) {
		std::cout << "Canot " << path_to_frames << std::endl;
		return 1;
	}

	unsigned int video_length_3;
	int dummy;
	in_frames >> video_length_3 >> dummy;

	std::vector<cv::Mat> frames(video_length_3);
	for(size_t i=0; i<frames.size(); ++i) {
		std::string frame_name;
		in_frames >> frame_name;

		frames[i] = cv::imread(root_dir + frame_name);
		if(frames[i].data == 0) {
			std::cout << "Cannot read " << (root_dir + frame_name) << std::endl;
			return 1;
		}
	}

	// check input
	if(video_length_3 != video_length) {
		std::cout << "Trajectory file is wrong"	<< std::endl;
		return 1;
	}
	if(video_length_3 != video_length_2) {
		std::cout << "Partition file is wrong" << std::endl;
		return 1;
	}

	// draw trajectories
	voxel_map_t pos_2_trajectory;
	draw_trajectories(trajectories, frames, pos_2_trajectory);

	//// show graphs
	const std::string current_frame_name("Current frame");
	cv::namedWindow(current_frame_name);

	unsigned int current_frame_number = 0;
	mouse_callback_input_t mouse_callback_input(current_frame_number, pos_2_trajectory, trajectories, partitions, graph_scale);
	cv::setMouseCallback(current_frame_name, show_graphs, &mouse_callback_input);

	for(;;) {
		cv::imshow(current_frame_name, frames[current_frame_number]);

		int c = cv::waitKey(0);
		if( (c & 255) == 27 ) { // if ESC
			return 0;
		}
		switch( (char)c) {
			case 'f':
				// Go to the next frame
				if(current_frame_number < video_length-1) {
					++current_frame_number;
				}
				break;
			case 'b':
				// Go to the previous frame
				if(current_frame_number > 0) {
					--current_frame_number;
				}
				break;
			case 'r':
				// Refresh
				cv::destroyAllWindows();
				cv::imshow(current_frame_name, frames[current_frame_number]);
				cv::setMouseCallback(current_frame_name, show_graphs, &mouse_callback_input);
				break;
		}
	}
	return 0;
}

static void show_graphs( int event, int x, int y, int, void * args)
{
	if(event == cv::EVENT_MOUSEMOVE) {
		return;
	}

	switch(event) {
		case cv::EVENT_LBUTTONDOWN: {
			mouse_callback_input_t * callback_input = (mouse_callback_input_t*)args;

			unsigned int current_frame = callback_input->_current_frame_number;
			int seleceted_traj_id = callback_input->_trajectory_id(x, y, current_frame);
			if(seleceted_traj_id == -1) { // trajectory is not selected
				return;
			}
			
			const trajectory_t & trajectory = callback_input->_trajectories[seleceted_traj_id];
			const partition_t & partition = callback_input->_partitions[seleceted_traj_id];

			cv::Mat xy_proj, xt_proj, yt_proj;
			draw_xy_projection(trajectory, partition, xy_proj, callback_input->_scale);
			draw_tx_projection(trajectory, partition, xt_proj, callback_input->_scale);
			draw_ty_projection(trajectory, partition, yt_proj, callback_input->_scale);

			cv::imshow("xy projection: " + std::to_string(seleceted_traj_id), xy_proj);
			cv::imshow("tx projection: " + std::to_string(seleceted_traj_id), xt_proj);
			cv::imshow("ty projection: " + std::to_string(seleceted_traj_id), yt_proj);
			break;
		}
		case cv::EVENT_RBUTTONDOWN: {
			// show 3D trajectory
			break;
		}
	}
}

void draw_xy_projection( const trajectory_t & tr, const partition_t & parts, cv::Mat & out, float scale)
{
	if( scale < 1 || scale > 100) {
	       	scale = 1;
	}
	const cv::Size border_pix(5,5);
	
	// Originally trajectorys` elements are float
	int min_x = floor(*std::min_element(tr._x.begin(), tr._x.end()));
	int max_x = ceil(*std::max_element(tr._x.begin(), tr._x.end()));
	int min_y = floor(*std::min_element(tr._y.begin(), tr._y.end()));
	int max_y = ceil(*std::max_element(tr._y.begin(), tr._y.end()));

	if( scale*std::max(max_x-min_x+1, max_y-min_y+1) > max_win_size ) {
		scale = max_win_size / std::max(max_x-min_x+1, max_y-min_y+1);
	}

	cv::Point zero = scale*cv::Point(min_x, min_y);
	cv::Size plot_size(scale*(max_x-min_x+1), scale*(max_y-min_y+1));
	cv::Size out_size = plot_size + border_pix*2;

	out.release();
	out = cv::Mat::zeros(out_size, CV_8UC3);
	draw_2D_axies(out, border_pix);

	cv::Mat plot_area(out, cv::Rect(border_pix.width, border_pix.height, plot_size.width, plot_size.height)); // TODO make more compact

	// Draw trajectory
	for(size_t i=0; i<tr.size()-1; ++i) {
		cv::Point from = scale*cv::Point(tr._x[i], tr._y[i]);
		cv::Point to = scale*cv::Point(tr._x[i+1], tr._y[i+1]);
		cv::line(plot_area, from-zero, to-zero, cv::Scalar(0,125,0));
	}
	// Show partitions
	for( unsigned int p : parts ) {
		cv::Point boundary = scale*cv::Point(tr._x[p], tr._y[p]);
		cv::circle(plot_area, boundary-zero, 2, cv::Scalar(0,0,255), -1);
	}
}
void draw_tx_projection( const trajectory_t & tr, const partition_t & parts, cv::Mat & out, float scale)
{
	if( scale < 1 || scale > 100) {
	       	scale = 1;
	}
	const cv::Size border_pix(5,5);
	
	// Originally trajectorys` elements are float
	int min_x = floor(*std::min_element(tr._x.begin(), tr._x.end()));
	int max_x = ceil(*std::max_element(tr._x.begin(), tr._x.end()));
	int start = tr._t.front();
	int end = tr._t.back();

	if( scale*std::max(end-start+1, max_x-min_x+1) > max_win_size ) {
		scale = max_win_size / std::max(end-start+1, max_x-min_x+1);
	}

	cv::Point zero = scale*cv::Point(start, min_x);
	cv::Size plot_size(scale*(end-start+1), scale*(max_x-min_x+1));
	cv::Size out_size = plot_size + border_pix*2;

	out.release();
	out = cv::Mat::zeros(out_size, CV_8UC3);
	draw_2D_axies(out, border_pix);

	cv::Mat plot_area(out, cv::Rect(border_pix.width, border_pix.height, plot_size.width, plot_size.height)); // TODO make more compact

	// Draw trajectory
	for(size_t i=0; i<tr.size()-1; ++i) {
		cv::Point from = scale*cv::Point(tr._t[i], tr._x[i]);
		cv::Point to = scale*cv::Point(tr._t[i+1], tr._x[i+1]);
		cv::line(plot_area, from-zero, to-zero, cv::Scalar(0,125,0));
	}
	// Show partitions
	for( unsigned int p : parts ) {
		cv::Point boundary = scale*cv::Point(tr._t[p], tr._x[p]);
		cv::circle(plot_area, boundary-zero, 2, cv::Scalar(0,0,255), -1);
	}
}
void draw_ty_projection( const trajectory_t & tr, const partition_t & parts, cv::Mat & out, float scale)
{
	if( scale < 1 || scale > 100) {
	       	scale = 1;
	}
	const cv::Size border_pix(5,5);
	
	// Originally trajectorys` elements are float
	int min_y = floor(*std::min_element(tr._y.begin(), tr._y.end()));
	int max_y = ceil(*std::max_element(tr._y.begin(), tr._y.end()));
	int start = tr._t.front();
	int end = tr._t.back();

	if( scale*std::max(end-start+1, max_y-min_y+1) > max_win_size ) {
		scale = max_win_size / std::max(end-start+1, max_y-min_y+1);
	}

	cv::Point zero = scale*cv::Point(start, min_y);
	cv::Size plot_size(scale*(end-start+1), scale*(max_y-min_y+1));
	cv::Size out_size = plot_size + border_pix*2;

	out.release();
	out = cv::Mat::zeros(out_size, CV_8UC3);
	draw_2D_axies(out, border_pix);

	cv::Mat plot_area(out, cv::Rect(border_pix.width, border_pix.height, plot_size.width, plot_size.height)); // TODO make more compact

	// Draw trajectory
	for(size_t i=0; i<tr.size()-1; ++i) {
		cv::Point from = scale*cv::Point(tr._t[i], tr._y[i]);
		cv::Point to = scale*cv::Point(tr._t[i+1], tr._y[i+1]);
		cv::line(plot_area, from-zero, to-zero, cv::Scalar(0,125,0));
	}
	// Show partitions
	for( unsigned int p : parts ) {
		cv::Point boundary = scale*cv::Point(tr._t[p], tr._y[p]);
		cv::circle(plot_area, boundary-zero, 2, cv::Scalar(0,0,255), -1);
	}
}

void draw_2D_axies(cv::Mat & region, const cv::Size & border)
{
	cv::Rect rect(border.width, border.height, region.size().width-2*border.width, region.size().height-2*border.height);
	cv::rectangle(region, rect, cv::Scalar(75, 75, 75));
}

void draw_trajectories(const std::vector<trajectory_t> & trajectories, std::vector<cv::Mat> & video, voxel_map_t & pos_2_trajectory)
{
	int video_length = video.size();
	int width = video[0].size().width; 
	int height = video[0].size().height;
			
	pos_2_trajectory.resize(width, height, video_length);

	// color scheme idea is taken from T. Brox dense trajectories
	std::vector<cv::Scalar> color_scheme(1024);
	for(int i=0; i<256; ++i) {
		color_scheme[i] = cv::Scalar(255, i, 0);
	}
	for(int i=0; i<256; ++i) {
		color_scheme[256+i] = cv::Scalar(255-i, 255, 0);
	}
	for(int i=0; i<256; ++i) {
		color_scheme[512+i] = cv::Scalar(0, 255, i);
	}
	for(int i=0; i<256; ++i) {
		color_scheme[768+i] = cv::Scalar(255, 255, 255);
	}

	for(unsigned int j=0; j<trajectories.size(); ++j) {
		for(size_t i=0; i<trajectories[j].size(); ++i) {
			int t = trajectories[j]._t[i]; 

			int ix = lroundf(trajectories[j]._x[i]); 
			int iy = lroundf(trajectories[j]._y[i]); 
			cv::Point p1, p2; 
			p1.x = (ix-1<0)? 0: ix-1; 
			p1.y = (iy-1<0)? 0: iy-1; 
			p2.x = (ix+1>=width)? width-1: ix+1; 
			p2.y = (iy+1>=height)? height-1: iy+1; 
			cv::rectangle(video[t], p1, p2, color_scheme[i*10], CV_FILLED); 

			for(short y = std::max(p1.y-2, 0); y <= std::min(p2.y+2, height-1); ++y) {
				for(short x = std::max(p1.x-2, 0); x <= std::min(p2.x+2, width-1); ++x) {
					pos_2_trajectory(x,y,t) = j;
				}
			}
		}
	}
}
