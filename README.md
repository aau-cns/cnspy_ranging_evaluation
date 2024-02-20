# cnspy_ranging_evaluation

A Python package to evaluate the two-way-ranging measurements between UWB modules in order to assess the accuracy.
The baseline (ground truth) ranges can be computed from a recorded 3D trajectory of the moving ranging devices (tags) and known positions of stationary devices (anchors). 

## Installation

Python 3.6 or greater is required. Inside this repo's directory, you may run
```
pip3 install .
```
or
``
pip3 install -e .
``
which installs the package in-place, allowing you make changes to the code without having to reinstall every time.

**This package is still in development. Once stable it should be sufficient to run:**
```commandline
pip3 install cnspy_ranging_evaluation
```

## Usage

This package contains different tools to evaluate the Two-Way-Ranging (TWR) measurements between pairs  <ID1, ID2> of devices. 

1. Record a bag file on the drone with all UWB ranging topics.
2. Specify a YAML configuration file, e.g, `cfgs_tags.yaml` (see below).
3. Run `TWR_ROSbag2CSV.py` on the measured bag file
```commandline 
python TWR_ROSbag2CSV.py --bagfile DH_A9_T2_loiter_spiral_8m_2023-09-02-23-36-02.bag --topics /a01/ranging /a02/ranging /a03/ranging /a04/ranging /a06/ranging /a07/ranging /a08/ranging /a09/ranging /a10/ranging /d01/tag1/ranging /d01/tag2/ranging --verbose --filenames all_measured_ranges.csv
```
4. Run `ROSBag_TrueRanges` to generate a clean (true) bag file of the obtained one
```commandline  
python ROSBag_TrueRanges.py
--bagfile_in DH_A9_T2_loiter_spiral_8m_2023-09-02-23-36-02.bag
--bagfile_out DH_A9_T2_loiter_spiral_8m_2023-09-02-23-36-02-true-ranges.bag
--topic_pose
/d01/mavros/vision_pose/pose
--cfg cfg_tags.yaml
--verbose
--std_range 0.0001
``` 
5. Run `TWR_ROSbag2CSV.py` on the clean (true) bag file
```commandline 
python TWR_ROSbag2CSV.py --bagfile ./DH_A9_T2_loiter_spiral_8m_2023-09-02-23-36-02-true-ranges.bag --topics /a01/ranging /a02/ranging /a03/ranging /a04/ranging /a05/ranging /a06/ranging /a07/ranging /a08/ranging /a09/ranging /a10/ranging  --verbose --filenames all_true_ranges.csv
```
6. Run `RangeEvaluation.py` using both csv files
```cmd
--fn_gt ./DH_A9_T2_loiter_spiral_8m/DH_A9_T2_loiter_spiral_8m_2023-09-02-23-36-02-true-ranges/all_true_ranges.csv
--fn_est ./DH_A9_T2_loiter_spiral_8m/DH_A9_T2_loiter_spiral_8m_2023-09-02-23-36-02/all_meas_ranges.csv
--result_dir .DH_A9_T2_loiter_spiral_8m/DH_A9_T2_loiter_spiral_8m_2023-09-02-23-36-02/bias_eval/
--UWB_ID1s 100 101 102 103 104 105 106 107 108 109 110
--UWB_ID2s 100 101 102 103 104 105 106 107 108  109 110
--plot
--show_plot
--plot_histograms
--relative_timestamp
--verbose
--max_range 15
--max_timestamp_difference 0.02
```

### YAML configuration file

YAML configuration file is in the form of:
```yaml
# relative position of the moving tags
rel_tag_positions: {100: [-0.19, 0.105, -0.07], 105: [0.05, -0.105, -0.07]}
tag_topics: {100: "/d01/tag1/ranging", 105: "/d01/tag2/ranging"}
anchor_topics: {101: "/a01/ranging", 102: "/a02/ranging", 103: "/a03/ranging", 104: "/a04/ranging", 106: "/a06/ranging", 107: "/a07/ranging", 108: "/a08/ranging", 109: "/a09/ranging", 110: "/a10/ranging"}
# relative position of the stationary anchors
abs_anchor_positions: {101: [-1.308, -4.140, 0.66], 102: [1.742,-4.147,1.881], 103: [2.914,2.081,2.172], 104: [0.18, -4.13, 3.242], 106: [-1.772, 0.943, 3.256], 107: [-2.021, 1.814, 1.732], 108: [-1.98, 0.069, 1.76], 109: [0.433, 4.105, 0.925], 110: [2.59, -0.2, 0.33]}
```

## Tools 

### ROSBag_TrueRanges
```commandline
cnspy_ranging_evaluation$ python ROSBag_TrueRanges.py -h
usage: ROSBag_TrueRanges.py [-h] --bagfile_in BAGFILE_IN [--bagfile_out BAGFILE_OUT] --topic_pose TOPIC_POSE --cfg CFG [--verbose] [--std_range STD_RANGE] [--bias_offset BIAS_OFFSET] [--bias_range BIAS_RANGE] [--use_header_timestamp]

ROSBag_TrueRanges: extract a given pose topic and compute ranges to N abs_anchor_positions and M rel_tag_positions, which is stored into a CSV file

optional arguments:
  -h, --help            show this help message and exit
  --bagfile_in BAGFILE_IN
                        input bag file
  --bagfile_out BAGFILE_OUT
                        output bag file
  --topic_pose TOPIC_POSE
                        desired topic
  --cfg CFG             YAML configuration file describing the setup: {rel_tag_positions, abs_anchor_positions}
  --verbose
  --std_range STD_RANGE
                        standard deviation of generated measurements: z = d + white_noise(std_range)
  --bias_offset BIAS_OFFSET
                        constant offset added to generated measurements: z = d + bias_offset
  --bias_range BIAS_RANGE
                        range-based biased multiplied to generated measurements: z = bias_range * d
  --use_header_timestamp
                        overwrites the bag time with the header time stamp
```

YAML configuration file is in the form of:
```yaml
# relative position of the moving tags
rel_tag_positions: {0:[-0.09, 0.04, -0.045]}
# relative position of the stationary anchors
abs_anchor_positions: {1:[-1.306, -4.146, 0.662], 2:[1.748,-4.173,1.878], 3:[2.928,2.47,2.153]}

```

### ROSBAg_Pose2Ranges

```commandline
cnspy_ranging_evaluation$ python ROSBag_Pose2Ranges.py -h
usage: ROSBag_Pose2Ranges.py [-h] --bagfile BAGFILE --topic TOPIC --cfg CFG [--filename FILENAME] [--result_dir RESULT_DIR] [--verbose]

ROSBag_Pose2Ranges: extract a given pose topic and compute ranges to N abs_anchor_positions and M rel_tag_positions, which is stored into a CSV file

optional arguments:
  -h, --help            show this help message and exit
  --bagfile BAGFILE     input bag file
  --topic TOPIC         desired topic
  --cfg CFG             YAML configuration file describing the setup: {rel_tag_positions, abs_anchor_positions}
  --filename FILENAME   csv filename of corresponding topic
  --result_dir RESULT_DIR
                        directory to store results [otherwise bagfile name will be a directory]
  --verbose
```


Example:
```commandline
ROSBag_Pose2Ranges.py --bagfile ../test/sample_data//uwb_calib_a01_2023-08-31-21-05-46.bag --topic /d01/mavros/vision_pose/pose --cfg ../test/sample_data/config.yaml --verbose
```

### TWR_ROSbag2CSV

```commandline
cnspy_ranging_evaluation$ python TWR_ROSbag2CSV.py -h
usage: TWR_ROSbag2CSV.py [-h] [--bagfile BAGFILE] [--topics [TOPICS [TOPICS ...]]] [--filenames [FILENAMES [FILENAMES ...]]] [--result_dir RESULT_DIR] [--verbose]

TWR_ROSbag2CSV: extract and store given topics of a rosbag into a CSV file

optional arguments:
  -h, --help            show this help message and exit
  --bagfile BAGFILE     input bag file
  --topics [TOPICS [TOPICS ...]]
                        desired topics
  --filenames [FILENAMES [FILENAMES ...]]
                        csv filename of corresponding topic
  --result_dir RESULT_DIR
                        directory to store results [otherwise bagfile name will be a directory]
  --verbose
```

### RangeEvaluation

```commandline
cnspy_ranging_evaluation$ python RangeEvaluation.py -h
usage: RangeEvaluation.py [-h] [--fn_gt FN_GT] [--fn_est FN_EST] [--result_dir RESULT_DIR] [--UWB_ID1s UWB_ID1S [UWB_ID1S ...]] [--UWB_ID2s UWB_ID2S [UWB_ID2S ...]] [--prefix PREFIX] [--max_timestamp_difference MAX_TIMESTAMP_DIFFERENCE] [--subsample SUBSAMPLE] [--plot] [--save_plot] [--show_plot]
                          [--relative_timestamps] [--remove_outliers] [--verbose] [--max_range MAX_RANGE] [--range_error_val RANGE_ERROR_VAL] [--label_timestamp LABEL_TIMESTAMP] [--label_range LABEL_RANGE] [--label_ID1 LABEL_ID1] [--label_ID2 LABEL_ID2] [--plot_timestamps] [--plot_ranges]
                          [--plot_ranges_sorted] [--plot_errors] [--plot_histograms]

RangeEvaluation: evaluate and estimated and true pairwise ranges

optional arguments:
  -h, --help            show this help message and exit
  --fn_gt FN_GT         input ground-truth trajectory CSV file
  --fn_est FN_EST       input estimated trajectory CSV file
  --result_dir RESULT_DIR
                        directory to store results [otherwise bagfile name will be a directory]
  --UWB_ID1s UWB_ID1S [UWB_ID1S ...]
                        ID of TX
  --UWB_ID2s UWB_ID2S [UWB_ID2S ...]
                        ID of RX
  --prefix PREFIX       prefix in results
  --max_timestamp_difference MAX_TIMESTAMP_DIFFERENCE
                        Max difference between associated timestampes (t_gt - t_est)
  --subsample SUBSAMPLE
                        subsampling factor for input data (CSV)
  --plot
  --save_plot
  --show_plot
  --relative_timestamps
  --remove_outliers
  --verbose
  --max_range MAX_RANGE
                        range that classifies as outlier
  --range_error_val RANGE_ERROR_VAL
                        value assigned to outlier
  --label_timestamp LABEL_TIMESTAMP
                        timestamp label in CSV
  --label_range LABEL_RANGE
                        range label in CSV
  --label_ID1 LABEL_ID1
                        ID1 label in CSV
  --label_ID2 LABEL_ID2
                        ID2 label in CSV
  --plot_timestamps
  --plot_ranges
  --plot_ranges_sorted
  --plot_errors
  --plot_histograms
```

```commandline
--fn_gt
/tmp/uwb_dataset/DH_A9_T2_loiter_spiral_1m_2m2/loiter_spiral_h1_to_2_2_2023-09-19-11-51-05-true-ranges/all_true_ranges.csv
--fn_est
/tmp/uwb_dataset/DH_A9_T2_loiter_spiral_1m_2m2/loiter_spiral_h1_to_2_2_2023-09-19-11-51-05/all_measured_ranges.csv
--result_dir
/tmp/uwb_dataset/DH_A9_T2_loiter_spiral_1m_2m2/loiter_spiral_h1_to_2_2_2023-09-19-11-51-05/bias_eval/
--UWB_ID1s 100 101 102 103 104 105 106 107 108 109 110
--UWB_ID2s 100 101 102 103 104 105 106 107 108  109 110
--plot
--show_plot
--plot_histograms
--relative_timestamp
--verbose
--max_range 15
--max_timestamp_difference 0.02
```

### CSV_StaticBiasAnalysis

```commandline
cnspy_ranging_evaluation$ python CSV_StaticBiasAnalysis.py -h
usage: CSV_StaticBiasAnalysis.py [-h] --csv_fn CSV_FN --cfg_fn CFG_FN [--verbose] [--label_timestamp LABEL_TIMESTAMP] [--label_range LABEL_RANGE] [--label_ID1 LABEL_ID1] [--label_ID2 LABEL_ID2]

CSV_StaticBiasAnalysis: extract a given pose topic and compute ranges to N abs_anchor_positions and M rel_tag_positions, which is stored into a CSV file

optional arguments:
  -h, --help            show this help message and exit
  --csv_fn CSV_FN       input bag file
  --cfg_fn CFG_FN       YAML configuration file describing the setup: {abs_anchor_positions}
  --verbose
  --label_timestamp LABEL_TIMESTAMP
                        timestamp label in CSV
  --label_range LABEL_RANGE
                        range label in CSV
  --label_ID1 LABEL_ID1
                        ID1 label in CSV
  --label_ID2 LABEL_ID2
                        ID2 label in CSV

```

YAML configuration file is in the form of:
```yaml
# relative position of the moving tags
rel_tag_positions: {0:[-0.09, 0.04, -0.045]}
# relative position of the stationary anchors
abs_anchor_positions: {1:[-1.306, -4.146, 0.662], 2:[1.748,-4.173,1.878], 3:[2.928,2.47,2.153]}

```