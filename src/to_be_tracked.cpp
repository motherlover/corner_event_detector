#include "corner_event_detector/to_be_tracked.h"

namespace corner_event_detector
{

ToBeTracked::ToBeTracked(void)
{	
	// Distribute the n corners over an arc centered at location 
	tracked_corners = Eigen::MatrixXd::Zero(ncorners,3); // init matrix	
	const double angular_step = 2*pi/ncorners;
	const double angular_correction = angular_step/2;
	for (int i=0; i<ncorners; i++) 
	{
		// Define corners
		tracked_corners(i,0) = center_x + (int)cos(angular_step - angular_correction) * radius;
		tracked_corners(i,1) = center_y + (int)sin(angular_step - angular_correction) * radius;		
		tracked_corners(i,2) = init_ts;
		// Define offset from center per corner
		corner_offset[i] = radius;
	}
	// Define object center
	object_center[0] = center_x;
	object_center[1] = center_y; 

}

ToBeTracked::~ToBeTracked(void)
{
}

bool ToBeTracked::ClassifyEvent(int x, int y)
{
	// Loop through tracked corners to see if the event lies within the \
	search window of one of them
	bool in_window = false;
	for (int i=0; i<ncorners; i++) 
	{
		if (x > tracked_corners(i,0)-window_size && \
			x < tracked_corners(i,0)+window_size && \
			y > tracked_corners(i,1)-window_size && \
			y < tracked_corners(i,1)+window_size)
		{
			in_window = true;
			break;
		}
	}
	return in_window;
}

bool ToBeTracked::ClassifyCorner(int x, int y)
{
	// Calculate offset
	int offset = (int) sqrt ((object_center[0] - x)*(object_center[0] - x) \
		 + (object_center[1] - y)*(object_center[1] - y));
	// Loop through tracked corners to see if the corner is an outlier \
	with respect to the corners classified as belonging to the object
	bool in_window = true;	
	for (int i=0; i<ncorners; i++) 
	{
		if (offset > 2*corner_offset[i])
		{
			in_window = false;
			break;
		}
	}
	return in_window;
}

void ToBeTracked::Update(int x, int y, double t)
{	
	// Find smallest distance to a corner
	int d_min = 2*window_size; // Initialize for loop
	int i_match = 0; // Matching corner
	for (int i=0; i<ncorners; i++)
	{
		// Calculate distance of corner to corners being tracked
		int distance = (int) sqrt ((x - tracked_corners(i,0))*(x - tracked_corners(i,0)) \
		 + (y - tracked_corners(i,1))*(y - tracked_corners(i,1)));
		// Check for the corner with shortest distance
		if (d_min > distance)
		{
			i_match = i;
			d_min = distance;
		}
	}
	// Update center
	object_center[0] = (int) (object_center[0]*ncorners-tracked_corners(i_match,0)+x)/ncorners;
	object_center[1] = (int) (object_center[1]*ncorners-tracked_corners(i_match,1)+x)/ncorners;	
	// Update offsets
	int sum_offset = 0;
	for (int i=0; i<ncorners; i++)
	{
		new_corner_offset = (int) sqrt ((object_center[0] - tracked_corners(i,0))*(object_center[0] - tracked_corners(i,0)) \
		 + (object_center[1] - tracked_corners(i,1))*(object_center[1] - tracked_corners(i,1)));	
		corner_offset[i] = new_corner_offset;
		sum_offset += new_corner_offset; 	
	}

	// Update the location of the matched corner
	int new_x = (x + tracked_corners(i_match,0))/2;
	int new_y = (y + tracked_corners(i_match,1))/2;
	tracked_corners(i_match,0) = new_x;
	tracked_corners(i_match,1) = new_y;
	tracked_corners(i_match,2) = t;
	// Update mean offset
	mean_offset = (int) sum_offset/ncorners;

}

void ToBeTracked::ActiveCornersUpdate(double t)
{
	for (int i=0; i<ncorners; i++)
	{
		// Check if still active
		if (t - tracked_corners(i,2) > dt_max)
		{
			// If not then update
			UpdateInactiveCorner(i,t)
		}
	}
} 

void ToBeTracked::UpdateInactiveCorner(int i, double t)
{
	// Place corner at average offset at random angle
	angle = ((double) rand() / (RAND_MAX)) * 2 * pi;
	x_reinit = (int) cos(angle) * mean_offset;
	y_reinit = (int) sin(angle) * mean_offset;
	tracked_corners(i,0) = x_reinit;
	tracked_corners(i,1) = y_reinit;
	tracked_corners(i,2) = t;
} 



}// namespace