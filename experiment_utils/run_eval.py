import os
import argparse

if __name__ == '__main__':
    # argparser to get dir
    
    argparser = argparse.ArgumentParser(description='Run evaluation on the output directories.')
    
    argparser.add_argument('--input_dir', type=str, required=True, help='Path to the output directory.')
    argparser.add_argument('--output_dir', type=str, required=True, help='Path to the output directory.')
    argparser.add_argument('--exp_name', type=str, required=True, help='Experiment planner.')
    argparser.add_argument('--past_n_trials', type=int, required=True, help='Past n trials.')
    
    args = argparser.parse_args()
    
    input_dir = args.input_dir
    output_exp_dir = args.output_dir
    
    exp_name = args.exp_name
    
    past_n_trials = args.past_n_trials
    
    # get relevant config files
    # dir = 'outputs/depth-gaussian-splatting'
    
    output_exp_dir = 'experiments'
    
    files = sorted(os.listdir(input_dir))
    print(files)
    
    full_exp_dir = os.path.join(output_exp_dir, exp_name)
    
    os.makedirs(full_exp_dir, exist_ok=True)
    
    amt = 0
    
    for file in files[::-1]:
        config_yml_path = os.path.join(input_dir, file, 'config.yml')
        print(config_yml_path)
        exp_json = exp_name + f'_{amt+1}.json'
        full_exp_json = os.path.join(full_exp_dir, exp_json)
        print(exp_json)
        full_cmd = f'ns-eval --load-config={config_yml_path} --output-path={full_exp_json}'
        print(full_cmd)
        
        os.system(full_cmd)
        
        full_cmd = f'ns-render dataset --load-config={config_yml_path} --output-path={exp_name}_renders'
        print(full_cmd)
        
        os.system(full_cmd)
        
        # run render
        
        amt += 1
        if amt == past_n_trials:
            break
