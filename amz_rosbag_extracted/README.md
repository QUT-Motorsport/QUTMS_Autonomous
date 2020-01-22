## AMZ's GPS data

We can see all the topics published by AMZ sensors by doing:
```rosbag info <_file.bag_>``` 


Using AMZ's rosbag file, extraction of a topic can be done by doing:
```rostopic echo -b _file.bag_ -p /gps > gps.txt```

This is later parsed as CSV and plotted using matlab (see .m and .png)


