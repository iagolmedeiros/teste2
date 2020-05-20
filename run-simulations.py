import sem
import argparse

ns_path = './'
script = 'lte'
campaign_dir = ns_path + 'sem'
results_dir = ns_path + 'results'

parser = argparse.ArgumentParser(description='SEM script')
parser.add_argument('-o', '--overwrite', action='store_true',
                    help='Overwrite previous campaign')
args = parser.parse_args()

campaign = sem.CampaignManager.new(ns_path, script, campaign_dir,
            overwrite=args.overwrite, check_repo=True)
print(campaign)

param_combinations = {
    'algo' : ['kmeans', 'meanshift', 'dbscan', 'hdbscan'],
    'enablePrediction' : 'true',
    'numUAVs' : 3,
    'numUes' : 75,
    'seedValue' : 10000,
    'useCa' : 'false'
}

campaign.run_missing_simulations(sem.list_param_combinations(param_combinations),10)

result_param = { 
    'algo' : ['kmeans', 'meanshift', 'dbscan', 'hdbscan']
}

campaign.save_to_folders(result_param, results_dir, 10)
