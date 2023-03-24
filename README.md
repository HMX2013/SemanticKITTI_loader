# SemanticKITTI Dataset Loader
A ROS package to load semanticKITTI dataset

## Data Preparation

### SemanticKITTI
```
./
├── 
├── ...
└── path_to_data_shown_in_config/
    ├──sequences
        ├── 00/           
        │   ├── velodyne/	
        |   |	├── 000000.bin
        |   |	├── 000001.bin
        |   |	└── ...
        │   └── labels/ 
        |       ├── 000000.label
        |       ├── 000001.label
        |       └── ...
        ├── 08/ # for validation
        ├── 11/ # 11-21 for testing
        └── 21/
	    └── ...
```

## How to use
In the launch file, you have three parameters to set. \
"fps" represents the frequency of the published topic \
change the /data_dir to your dataset file path


    <rosparam param="/seq">"00"</rosparam>
    <rosparam param="/fps">10</rosparam>
    <rosparam param="/data_dir">"/home/semanticKITTI/sequences"</rosparam>


## Results
![demo_1](demo/demo_01.gif)
