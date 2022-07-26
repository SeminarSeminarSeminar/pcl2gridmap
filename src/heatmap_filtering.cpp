
#include "header/heatmap_filtering.hpp"
#include <algorithm>

#define grid_size 511

void decode_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered, int thres)
{
    unsigned int PC_size = cloud_filtered->size();

	std::vector<double> x_val_vec;
	std::vector<double> z_val_vec;
	for(int i=0; i<cloud_filtered->size(); i++)
	{
		x_val_vec.push_back(cloud_filtered->points[i].x);
		z_val_vec.push_back(cloud_filtered->points[i].z);
	}
	auto x_val_max = std::max_element(std::begin(x_val_vec), std::end(x_val_vec)); // C++11
	auto z_val_max = std::max_element(std::begin(z_val_vec), std::end(z_val_vec)); // C++11
	auto x_val_min = std::min_element(std::begin(x_val_vec), std::end(x_val_vec)); // C++11
	auto z_val_min = std::min_element(std::begin(z_val_vec), std::end(z_val_vec)); // C++11

	double max_x = *x_val_max;
	double min_x = *x_val_min;
	double max_z = *z_val_max;
	double min_z = *z_val_min;

	double scale_x = (double)grid_size/(max_x-min_x);
	double scale_z = (double)grid_size/(max_z-min_z);
	double bias_x = -min_x*scale_x;
	double bias_z = -min_z*scale_z;

	static unsigned char grid_array[grid_size+1][grid_size+1];
	static unsigned char grid_bound[grid_size+1][grid_size+1];
	static unsigned char grid_final[grid_size+1][grid_size+1];
	for(int z_idx=0; z_idx<grid_size+1; z_idx++)
		for(int x_idx=0; x_idx<grid_size+1; x_idx++)
		{
			grid_array[z_idx][x_idx]=0;
			grid_bound[z_idx][x_idx]=0;
			grid_final[z_idx][x_idx]=0;
		}


	int cut_size = 26;
	int bound_size = (int)((grid_size+1)/cut_size);
	std::cout<<"bound_szie : "<<bound_size<<std::endl;

	for(int i=0; i<PC_size; i++)
	{
		int quan_x_pos = (int)(scale_x*(x_val_vec[i])+bias_x);
		int quan_z_pos = (int)(scale_z*(z_val_vec[i])+bias_z);
		assert(quan_x_pos >=0 && quan_z_pos >= 0);
		assert(quan_x_pos<=512 && quan_z_pos<=512);
		grid_array[quan_z_pos][quan_x_pos]=255;
	}	

	#if 1
	for(int z_idx=0; z_idx<grid_size+1; z_idx++)
		for(int x_idx=0; x_idx<grid_size+1; x_idx++)
		{
			grid_bound[z_idx][x_idx]=0;
		}

	for(int row=0; row<grid_size+1; row++)
	{
		for(int col=0; col<grid_size+1; col++)
		{
			if( col % bound_size == 0 || col == grid_size)
				grid_bound[row][col]=255;
		}
	}
	std::vector<int> left_col;
	std::vector<int> right_col;
	for(int row_block=0; row_block<cut_size; row_block++)
	{
		for(int col_block=0; col_block<cut_size; col_block++)
		{
			for(int row=row_block*bound_size; row<(row_block+1)*bound_size; row++)
			{
				bool find_left=false;
				bool find_right=false; // 252 : 255
				for(int col=col_block*bound_size; col<(col_block+1)*bound_size; col++)
				{
					if(grid_array[row][col]>0)
					{
						if( find_left == false)
						{
							find_left=true;
							left_col.push_back(col);
						}
					}
				}
				for( int col=(col_block+1)*bound_size-1; col >= col_block*bound_size; col--)
				{
					if( grid_array[row][col]>0)
					{
						if( find_right == false)
						{
							find_right=true;
							right_col.push_back(col);
						}
					}
				}
				if( left_col.size() == right_col.size() )
				{
					for(int i=0; i<left_col.size(); i++)
					{
						for(int inner_col=left_col[i]; inner_col<=right_col[i]; inner_col++)
							grid_final[row][inner_col]=255;
					}
				}
				left_col.clear();
				right_col.clear();
			}
		}
	}
	#endif

	#if 1
	for(int z_idx=0; z_idx<grid_size+1; z_idx++)
		for(int x_idx=0; x_idx<grid_size+1; x_idx++)
		{
			grid_bound[z_idx][x_idx]=0;
		}

	for(int row=0; row<grid_size+1; row++)
	{
		for(int col=0; col<grid_size+1; col++)
		{
			if( row % bound_size == 0 || row == grid_size)
				grid_bound[row][col]=255;
		}
	}
	std::vector<int> up_row;
	std::vector<int> bt_row;
	for(int row_block=0; row_block<cut_size; row_block++)
	{
		for(int col_block=0; col_block<cut_size; col_block++)
		{
			for(int col=col_block*bound_size; col<(col_block+1)*bound_size; col++)
			{
				bool find_up=false;
				bool find_bt=false; // 252 : 255
				for(int row=row_block*bound_size; row<(row_block+1)*bound_size; row++)
				{
					if(grid_final[row][col]>0)
					{
						if( find_up == false)
						{
							find_up=true;
							up_row.push_back(row);
						}
					}
				}
				for( int row=(row_block+1)*bound_size-1; row >= row_block*bound_size; row--)
				{
					if( grid_final[row][col]>0)
					{
						if( find_bt == false)
						{
							find_bt = true;
							bt_row.push_back(row);
						}
					}
				}
				if( up_row.size() == bt_row.size() )
				{
					for(int i=0; i<up_row.size(); i++)
					{
						for(int inner_row=up_row[i]; inner_row<=bt_row[i]; inner_row++)
							grid_final[inner_row][col]=255;
					}
				}
				up_row.clear();
				bt_row.clear();
			}
		}
	}

	#endif

	#if 0
	int first_col_pos=-1;
	int second_col_pos=-1;
	for(int row=0; row<grid_size+1; row++)
	{
		for(int col=0; col<grid_size+1; col++)
		{
			if( first_col_pos==-1)
			{
				if( grid_final[row][col]>0)
					first_col_pos=col;
			}
			else
			{
				if( grid_final[row][col]==0 && first_col_pos != -1)
				{
					second_col_pos=col;
					grid_final[row][((first_col_pos+second_col_pos)/2)]=255;
					first_col_pos=-1;
					second_col_pos=-1;
				}
			}
		}
	}
	int first_row_pos=-1;
	int second_row_pos=-1;
	for(int col=0; col<grid_size+1; col++)
	{
		for(int row=0; row<grid_size+1; row++)
		{
			if( first_row_pos==-1)
			{
				if( grid_final[row][col]>0)
					first_row_pos=row;
			}
			else
			{
				if( grid_final[row][col]==0 && first_row_pos != -1)
				{
					second_row_pos=row;
					grid_final[((first_row_pos+second_row_pos)/2)][col]=255;
					first_row_pos=-1;
					second_row_pos=-1;
				}
			}
		}
	}

	#endif
	cv::Mat img_mat = cv::Mat(cv::Size(grid_size+1,grid_size+1), CV_8UC1, &grid_final);
	cv::imshow("test", img_mat);
	cv::waitKey(-1);

 	std::cout<<"Decode complete"<<std::endl;
	return;
}
