Basic MongoDB configuration
---------------------------

```bash
# Set permissions to log folder
sudo chown -R mongodb:mongodb /var/log/mongodb
# Set permissions to db folder
sudo chown -R mongodb:mongodb /var/lib/mongodb
```

Troubleshooting
---------------

When you run `sudo service mongodb status` you’ll get `mongodb stop/waiting` instead of `mongodb start/running`.	

This condition is largely due to an unclean shutdown, and results in the creation of a lockfile `/var/lib/mongodb/mongod.lock`.

```bash
# Remove lock file
sudo rm /var/lib/mongodb/mongod.lock
# Start and repair mongo
# Set `journal=false` at `/etc/mongodb.conf`
sudo -u mongodb mongod -f /etc/mongodb.conf --repair
```
Now when you run `sudo service mongodb start`.  It will report `mongodb start/running ...`.

Reinstall

```bash
# Remove mongodb
sudo apt-get purge mongodb
# Remove config and log folders
sudo rm -rf /var/log/mongodb
sudo rm -rf /var/lib/mongodb
# Install again (usually moveit get removed)
sudo apt-get install mongodb mongodb-clients mongodb-dev mongodb-server python-pymongo python-pymongo-ext
sudo apt-get install ros-indigo-moveit-full
# Set permissions to log folder
sudo chown -R mongodb:mongodb /var/log/mongodb
# Set permissions to db folder
sudo chown -R mongodb:mongodb /var/lib/mongodb
# Restart
sudo service mongodb restart
```

Configure at start up

```bash
sudo update-rc.d mongodb defaults
sudo update-rc.d mongodb enable
```

