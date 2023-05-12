import os
import yaml
import glob
import tqdm

from python_scripts.simulation import step_simulation
from python_scripts.utilities import merge_config, save_results
from commonroad.common.file_reader import CommonRoadFileReader

if __name__ == "__main__":
    skip_existing = True            # Skips any scenarios that already have existing results.
    scenario_path = "/home/jvarvn/Documents/Implementations/occlusion_tracking/scenarios/highd_scenarios"
    results_path = "/home/jvarvn/Documents/Implementations/occlusion_tracking/results/highd_simulations_pickle_new"

    path_xml = os.path.join(scenario_path, "*.xml")
    path_yaml = os.path.join(scenario_path, "*.yaml")

    xml_list = sorted(glob.glob(path_xml))
    yaml_list = sorted(glob.glob(path_yaml))

    with open("/home/jvarvn/Documents/Implementations/occlusion_tracking/scenarios/highd_config.yaml") as file:
        global_config = yaml.load(file, Loader=yaml.FullLoader)

    for xml_file, yaml_file in tqdm.tqdm(zip(xml_list, yaml_list), total=len(xml_list), desc="Iterating over simulations", position=0):
        scenario1, _ = CommonRoadFileReader(xml_file).open()
        scenario2, _ = CommonRoadFileReader(xml_file).open()
        with open(yaml_file) as file:
            scenario_config = yaml.load(file, Loader=yaml.FullLoader)
        config = merge_config(global_config, scenario_config)

        if skip_existing and os.path.isfile(os.path.join(results_path, str(scenario1.scenario_id))):
            continue

        try:
            config['occlusion_params']['ideal_tracking_enabled'] = False
            config['occlusion_params']['velocity_tracking_enabled'] = False
            untracked_results = step_simulation(scenario2, config)

            config['occlusion_params']['velocity_tracking_enabled'] = True
            tracked_results = step_simulation(scenario1, config)

            config['occlusion_params']['ideal_tracking_enabled'] = True
            ideal_results = step_simulation(scenario1, config)

            save_path = os.path.join(results_path, str(scenario1.scenario_id))
            save_results(save_path, ideal_results, tracked_results, untracked_results, scenario1, scenario_config)
        
        except Exception as e:
            print(e, scenario1.scenario_id)
