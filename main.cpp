 #include "ros/ros.h"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

typedef long long int u64;


using namespace std;

void write_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{

    unsigned int bram_size = 0x20000;
    off_t bram_x = 0xA0000000; // physical base address
    off_t bram_y = 0xA0020000; // physical base address
    off_t bram_z = 0xA0040000; // physical base address

    u64 *bram_x_ptr;
    u64 *bram_y_ptr;
    u64 *bram_z_ptr;

    int fd;
    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) != -1) {
        bram_x_ptr = (u64 *)mmap(NULL, bram_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, bram_x);
        bram_y_ptr = (u64 *)mmap(NULL, bram_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, bram_y);
        bram_z_ptr = (u64 *)mmap(NULL, bram_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, bram_z);
        int i =0;
        int j = 1;
        bram_x_ptr[0]= cloud->size();
        int a_16points_x=0;
        int a_16points_y=0;
        int a_16points_z=0;

        for (const auto& point: *cloud)
         {
            if ( i == 0)
            {
                a_16points_x = (int16_t)(point.x*100);
                a_16points_y = (int16_t)(point.y*100);
                a_16points_z = (int16_t)(point.z*100);

            }
            else {
                a_16points_x = a_16points_x +((int16_t)(point.x*100)<<(16*i));
                a_16points_y = a_16points_y +((int16_t)(point.y*100)<<(16*i));
                a_16points_z = a_16points_z +((int16_t)(point.z*100)<<(16*i));

            }

                if(i >= 15)
                {
                  cout<< "x: " <<hex<<a_16points_x<<" ; y: "<< hex<<a_16points_y<< " ; z: "<<hex<<a_16points_z<<endl;
                  bram_x_ptr[j]= a_16points_x;
                  bram_y_ptr[j]= a_16points_y;
                  bram_z_ptr[j]= a_16points_z;
                  j++;
                  i = 0;
                  a_16points_x=0;
                }
                i++;
        }
        j++;
        bram_x_ptr[j]= a_16points_x;
        bram_y_ptr[0]=j;
        close(fd);
        cout<< "Point cloud sended"<<endl;
        return;
    }
    cout << "error reading mem";

}



pcl::PointCloud<pcl::PointXYZI> read_pointcloud()
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    unsigned int bram_size = 0x20000;
    off_t bram_x = 0x80000000; // physical base address
    off_t bram_y = 0x82000000; // physical base address
    off_t bram_z = 0x84000000; // physical base address

    u64 *bram_x_ptr;
    u64 *bram_y_ptr;
    u64 *bram_z_ptr;

    int fd;
    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) != -1) {
        bram_x_ptr = (u64 *)mmap(NULL, bram_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, bram_x);
        bram_y_ptr = (u64 *)mmap(NULL, bram_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, bram_y);
        bram_z_ptr = (u64 *)mmap(NULL, bram_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, bram_z);
        int j = bram_y_ptr[0];
        cout << "total positions stored in bram:"<<j<<endl;
        int size = bram_x_ptr[0];
        cout << "point cloud size:"<<size<<endl;
        for (int i =1 ; i <=j;i++)
        {
            string out_x = to_string(bram_x_ptr[i]);
            string out_y = to_string(bram_y_ptr[i]);
            string out_z = to_string(bram_z_ptr[i]);
            cout << "found in memory:"<<endl<<"x: "<<out_x<<" ; y: "<<out_y<<" ; z: "<<out_z<<endl;
            //TODO depending on the result
        }
        close(fd);

    }

    return cloud;
}

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);


     ros::init (argc, argv, "alfa_node");
       if (!ros::master::check()) {
           cout <<"Failed to inicialize ros"<<endl;
       }

   while(1)
   {
       cout<< "1-> load point cloud to bram; 2-> read point cloud; 3-> test write; 4 ->test read"<<endl;
       int choice;
       cin >> choice;
       switch (choice) {
       case 1:
       {
           cout <<"point cloud path:"<<endl;
           string path;
           cin >> path;
           if (pcl::io::loadPCDFile<pcl::PointXYZI> (path, *cloud) == -1) //* load the file
           {
             PCL_ERROR ("Couldn't read file pointcloudl \n");
            // return (-1);
           }else
           write_pointcloud(cloud);
       }
           break;
       case 2:
           read_pointcloud();
           break;
       case 3:
       {
           int pos;
           cout << "enter position to write:"<<endl;
           cin >> pos;
           cout << "enter value to write: "<<endl;
           int value;
           cin>> value;
           unsigned int bram_size = 0x20000;
           int fd;
           off_t bram_x = 0x80000000; // physical base address
           u64 *bram_x_ptr;

           if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) != -1) {
               bram_x_ptr = (u64 *)mmap(NULL, bram_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, bram_x);
               bram_x_ptr[pos]=value;
               cout<< "value saved";
               close(fd);

           }
           else {
               cout<<"error opening mem";
           }
       }
           break;
       case 4:
       {
           int pos;
           cout << "enter position to read:"<<endl;
           cin >> pos;
           unsigned int bram_size = 0x20000;
           int fd;
           off_t bram_x = 0x80000000; // physical base address
           u64 *bram_x_ptr;
           int value;
           if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) != -1) {
               bram_x_ptr = (u64 *)mmap(NULL, bram_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, bram_x);
               value = bram_x_ptr[pos];
               cout<< "value readed: "<<value<<endl;
               close(fd);

           }
           else {
               cout<<"error opening mem";
           }
       }
           break;
       default:
           cout << "choice unknown"<<endl;
           break;
       }
   }

}
