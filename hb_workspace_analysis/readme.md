# MongoDB

We use MongoDB for database storage, this document it's about basic configuration and troubleshooting related with MongoDB.

### Database export

For export a MongoDB database (i.e. `workspace_analysis`) use:
```bash
mongodump -d workspace_analysis
```
This command create a `dump/workspace_analysis` folder structure with all database data.

### Database import

For import a MongoDB database (i.e. `workspace_analysis`) use:
```bash
mongorestore workspace_analysis
```

## Basic configuration

MongoDB configuration file (Ubuntu 14.04) `/etc/mongodb.conf`.

### Enable remote connection

`bind_ip` is a MongoDB option that restricts connections to specifics IPs.
```bash
sudo nano /etc/mongodb.conf
# Comment (with # character) the bind_ip line.
```

### MongoDB Shell

Using the shell you can get database information and make different operations.

```bash
$ mongo
MongoDB shell version: 2.4.9
connecting to: test
> show dbs
local	0.078125GB
> use local
switched to db local
> db.stats()
{
	"db" : "local",
	"collections" : 2,
	"objects" : 5,
	"avgObjSize" : 728,
	"dataSize" : 3640,
	"storageSize" : 10493952,
	"numExtents" : 2,
	"indexes" : 0,
	"indexSize" : 0,
	"fileSize" : 67108864,
	"nsSizeMB" : 16,
	"dataFileVersion" : {
		"major" : 4,
		"minor" : 5
	},
	"ok" : 1
}
> db.dropDatabase()
{ "dropped" : "local", "ok" : 1 }
> show dbs
> 
```
To exit the shell, type `quit()` or use the <kbd>Ctrl</kbd>+<kbd>C</kbd> shortcut.

## Troubleshooting

### Folder permission

Some times used folders don't get permissions.

```bash
# Set permissions to log folder
sudo chown -R mongodb:mongodb /var/log/mongodb
# Set permissions to db folder
sudo chown -R mongodb:mongodb /var/lib/mongodb
```
### Service `mongodb stop/waiting`

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

### Reinstall

Reinstall everything is a common solution.

```bash
# Remove mongodb (include MoveIt)
sudo apt-get purge mongodb mongodb-clients mongodb-dev mongodb-server python-pymongo python-pymongo-ext ros-indigo-moveit-full
# Remove config and log folders
sudo rm -rf /var/log/mongodb
sudo rm -rf /var/lib/mongodb
sudo rm -rf /etc/mongodb.conf
sudo rm -rf /etc/mongod.conf
# Install again (include MoveIt)
sudo apt-get install mongodb mongodb-clients mongodb-dev mongodb-server python-pymongo python-pymongo-ext ros-indigo-moveit-full
# Restart
sudo service mongodb restart
# Set permissions to log folder
sudo chown -R mongodb:mongodb /var/log/mongodb
# Set permissions to db folder
sudo chown -R mongodb:mongodb /var/lib/mongodb
# Configure at start up
sudo update-rc.d mongodb defaults
sudo update-rc.d mongodb enable
```
