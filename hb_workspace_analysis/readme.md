Basic MongoDB configuration

```bash
# Set permissions to log folder
sudo chown -R mongodb:mongodb /var/log/mongodb
# Set permissions to db folder
sudo chown -R mongodb:mongodb /var/lib/mongodb
```
Troubleshooting

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
