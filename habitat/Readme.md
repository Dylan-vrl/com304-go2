# Steps to follow (once)

## Habitat-sim and conda env
```bash
conda create -n habitat python=3.9 cmake=3.14.0
conda activate habitat
conda install habitat-sim withbullet -c conda-forge -c aihabitat
```

## Habitat-lab local install
Clone latest version somewhere you know (not in this git repo), enter the directory
```bash
git clone git@github.com:facebookresearch/habitat-lab.git
cd habitat-lab
```
While inside the habitat-lab folder (important), execute in terminal:
```bash
pip install -e habitat-labs
pip install -e habitat-baselines
pip install -e habitat-hitl
```


## Testing
### Install other dependencies for examples
```bash
pip install pygame pybullet opencv-python
```
### Download datasets
(in the habitat folder of the repo)
```bash
python -m habitat_sim.utils.datasets_download --uids habitat_test_scenes --data-path data/
python -m habitat_sim.utils.datasets_download --uids habitat_test_pointnav_dataset --data-path data/
```

# Notes
- Each time close the session you need to reactivate the conda environment before starting
- The config files are stored by default in a config folder of your installation location `./habitat-lab/habitat/config`, 
it can be modified by specifying to the function