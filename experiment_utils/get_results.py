import json
import glob

import os

import numpy as np

def read_json_file(file_path):
    """Read a JSON file and return the data."""
    with open(file_path, 'r') as file:
        return json.load(file)

def compute_averages(json_files):
    """Compute averages of PSNR, SSIM, LPIPS, and depth MSE from multiple JSON files."""
    # Initialize sums and count
    num_files = len(json_files)
    
    
    show_new_results = False
    
    gt_mses = []
    gt_obj_mses = []
    
    psnrs = []
    ssims = []
    lpipss = []
    depth_mses = []
    supervised_depth_mses = []
    
    
    # Loop through each file and accumulate metrics
    for file_path in json_files:
        data = read_json_file(file_path)
        results = data['results']
        psnrs.append(results['psnr'])
        
        
        ssims.append(results['ssim'])
        lpipss.append(results['lpips'])
        
        if 'depth_mse' in results:
            depth_mses.append(results['depth_mse'])
            
        if 'supervised_depth_mse' in results:
            show_new_results = True
            supervised_depth_mses.append(results['supervised_depth_mse'])
            
        if 'gt_depth_mse' in results:
            gt_mses.append(results['gt_depth_mse'])
            
        if 'gt_object_depth_mse' in results:
            gt_obj_mses.append(results['gt_object_depth_mse'])
            
    avg_psnr = np.mean(psnrs)
    avg_ssim = np.mean(ssims)
    avg_lpips = np.mean(lpipss)
    
    avg_depth_mse = np.mean(depth_mses)
    
    avg_supervised_depth_mse = np.mean(supervised_depth_mses)
    
    avg_gt_depth_mse = np.mean(gt_mses)
    
    avg_gt_object_depth_mse = np.mean(gt_obj_mses)
    
    print("Average PSNR: ", avg_psnr, "+-", np.std(psnrs))
    print("Average SSIM: ", avg_ssim, "+-", np.std(ssims))
    print("Average LPIPS: ", avg_lpips, "+-", np.std(lpipss))
    
    
    # if avg_depth_mse > 0:
    #     print("Average Depth MSE: ", avg_depth_mse, "+-", np.std(depth_mses))
        
    # if avg_supervised_depth_mse > 0:
        # print("Average Supervised Depth MSE: ", avg_supervised_depth_mse, "+-", np.std(supervised_depth_mses))
        
    if avg_gt_depth_mse > 0:
        print("Average GT Depth MSE: ", avg_gt_depth_mse, "+-", np.std(gt_mses))
        
    if avg_gt_object_depth_mse > 0:
        print("Average GT Object Depth MSE: ", avg_gt_object_depth_mse, "+-", np.std(gt_obj_mses))
    
    
    
    

# Example usage
if __name__ == "__main__":
    # Assuming all your JSON files are in the 'data' directory and have a '.json' extension
    dirs = [x[0] for x in os.walk('experiments')]
    print(dirs)
    for directory in sorted(dirs):
        if directory == 'experiments':
            continue
        exp_name = directory.split('/')[1]
        if len(exp_name) == 0:
            continue
        # exp_name = exp_name[1:]
        json_files = glob.glob(f'{directory}/*.json')
        
        print()
        print(f'{exp_name}')
        compute_averages(json_files)
        print()        
        
        