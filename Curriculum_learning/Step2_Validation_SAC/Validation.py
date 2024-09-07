from sac import SAC
import Config
import Env
import time
import csv
import torch

def Run_Validation(model_name = 'SAC_7113epi_weight20'):
    # Environment
    env = Env.RobotArmControl()
    env.reset()

    # Agent
    agent = SAC(9, 3, Config)
    model_path = f'trained_models/{model_name}.tar'
    
    # agent.load_checkpoint(model_path,evaluate=False)

    checkpoint = torch.load(model_path)
    agent.policy.load_state_dict(checkpoint['model'])
    agent.policy_optim.load_state_dict(checkpoint['optimizer'])

    iteration_per_UoC = 100
    successrate_of_model = []

    for current_level in range(env.MAX_Level_Of_Point+1):

        env.Level_Of_Point = current_level
        successrate_of_UoC = []

        for _ in range(iteration_per_UoC):
            done = False
            env.reset()
            state = env.get_state()

            while not done:
                action = agent.select_action(state)  # action from policy

                env.action(action)
                time.sleep(Config.time_sleep_interval)
                next_state, reward, done, success = env.observation() # Step

                state = next_state

            successrate_of_UoC.append(success)

        successrate_of_model.append(successrate_of_UoC)

    save_successrate_to_csv(successrate_of_model, filename=f'validation_results/{model_name}.csv')
    

def save_successrate_to_csv(successrate_of_model, filename):
    successrate_percentages = [(sum(successrate) / len(successrate)) * 100 if len(successrate) > 0 else 0 for successrate in successrate_of_model]
    
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Level", "Success Rate (%)"])
        for idx, successrate in enumerate(successrate_percentages):
            writer.writerow([idx+1, successrate])

    print(f"Success rates saved to {filename}")

if __name__ == '__main__':
    Run_Validation('SAC_7113epi_weight20')