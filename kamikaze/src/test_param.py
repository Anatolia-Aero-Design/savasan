from utilities import set_params_for_dive, set_params_for_safe_fly,print_param
import time

if __name__ == '__main__':
    print_param()
    set_params_for_dive()
    print_param()
    time.sleep(2)
    set_params_for_safe_fly()