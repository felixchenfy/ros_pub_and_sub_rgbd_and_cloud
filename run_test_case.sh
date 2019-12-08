publish_images(){
    roslaunch ros_pub_and_sub_rgbd_and_cloud run_publisher.launch
}

subscribe_images(){
    rosrun ros_pub_and_sub_rgbd_and_cloud sub_rgbd_and_cloud.py \
        --base_dir $(rospack find ros_pub_and_sub_rgbd_and_cloud) \
        --config_file config/rgbd_pub_config.yaml
}

# https://stackoverflow.com/questions/3004811/how-do-you-run-multiple-programs-in-parallel-from-a-bash-script
# Run above 2 functions in parallel:
(trap 'kill 0' SIGINT; publish_images & subscribe_images)
