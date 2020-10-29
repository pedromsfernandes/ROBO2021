# include "../include/wall_following.h"

/**
@brief The main node function
@param argc [int] Number of input arguments
@param argv [char] The input arguments
@return int : 0 for success
**/
int main(int argc,char **argv)
{
  ros::init(argc, argv, "stdr_wall_following", ros::init_options::AnonymousName);
  stdr_reactive::WallFollowing obj(argc, argv);
  ros::spin();
  return 0;
}
