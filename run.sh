# CW
# cd build
./stereo_kitti ../Vocabulary/ORBvoc.txt ../Examples/Stereo/KITTI00-02.yaml /mnt/hgfs/code/data_odometry_gray/00/

# tum rgbd
./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM1.yaml /mnt/hgfs/code/data_tum_rgbd/rgbd_dataset_freiburg1_xyz/ ../Examples/RGB-D/associations/fr1_xyz.txt 

./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM1.yaml /mnt/hgfs/code/data_tum_rgbd/rgbd_dataset_freiburg1_room/ ../Examples/RGB-D/associations/fr1_room.txt 





#./build/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml /home/lgj/Documents/rgbd_dataset_freiburg3_long_office_household /home/lgj/Documents/rgbd_dataset_freiburg3_long_office_household/association.txt 
./build/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml /home/lgj/Documents/mydataset /home/lgj/Documents/mydataset/association.txt 
