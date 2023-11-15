#!/bin/bash

NFS_SERVER_IP="your_nfs_server_ip"
NFS_SHARE_PATH="/path/to/nfs/share"
LOCAL_MOUNT_POINT="/local/mount/point"

# Create the local mount point directory if it doesn't exist
sudo mkdir -p $LOCAL_MOUNT_POINT

# Check if the NFS share is already mounted
if grep -qs "$NFS_SERVER_IP:$NFS_SHARE_PATH" /etc/fstab; then
    echo "NFS share is already configured to be mounted on startup."
else
    # Append the NFS share entry to /etc/fstab
    echo "$NFS_SERVER_IP:$NFS_SHARE_PATH $LOCAL_MOUNT_POINT nfs defaults 0 0" | sudo tee -a /etc/fstab

    # Mount the NFS share
    sudo mount -a

    # Print a message indicating whether the mount was successful
    if [ $? -eq 0 ]; then
        echo "NFS share mounted successfully."
    else
        echo "Failed to mount NFS share. Check the configuration and try again."
    fi
fi
