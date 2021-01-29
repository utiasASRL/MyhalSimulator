#!/usr/bin/env python

import rospy
from src.dashboard import *
from os import listdir


def plots1():

    D = Dashboard()  # optionally we can specify the verbosity of the output messages. default is logger.INFO

    D.list_runs(30)

    #D.add_series('Point_Slam_filtered', localization_technique='pointslam', filter_status='true')

    #   2020-10-16-12-29-11 - 2020-10-16-14-56-40 Round 2 bis
    earliest_date = '2020-10-16-12-29-11'
    latest_date = '2020-10-16-14-56-40'

    #   2020-10-15-18-24-48 - 2020-10-15-20-28-02 Round 3 (not trained on real round 2)
    # earliest_date = '2020-10-15-18-24-48'
    # latest_date = '2020-10-15-20-28-02'

    # earliest_date = '2020-10-15-21-07-17'
    # latest_date = '2020-10-15-23-10-33'
    filter_status = 'true'
    tour = ''

    D.add_series('Sc1',
                 earliest_date=earliest_date,
                 latest_date=latest_date,
                 tour_names=tour,
                 scenarios='Sc1_few_wanderers_some_tables',
                 localization_technique='pointslam',
                 filter_status=filter_status)

    D.add_series('Sc2',
                 earliest_date=earliest_date,
                 latest_date=latest_date,
                 tour_names=tour,
                 scenarios='Sc2_crowd_wanderers_some_tables',
                 localization_technique='pointslam',
                 filter_status=filter_status)

    D.add_series('Sc3',
                 earliest_date=earliest_date,
                 latest_date=latest_date,
                 tour_names=tour,
                 scenarios='Sc3_some_followers_some_tables',
                 localization_technique='pointslam',
                 filter_status=filter_status)

    earliest_date = '2020-10-15-18-24-48'
    latest_date = '2020-10-15-20-28-02'
    D.add_series('Sc3_round3',
                 earliest_date=earliest_date,
                 latest_date=latest_date,
                 tour_names=tour,
                 scenarios='Sc3_some_followers_some_tables',
                 localization_technique='pointslam',
                 filter_status=filter_status)

    D.add_plot(TrajectoryPlot(only_gt=True))
    D.show()
    D.remove_plot(TrajectoryPlot)

    D.add_plot(TranslationError(True))
    D.show()
    D.remove_plot(TranslationError)

    D.add_plot(TEBoxPlot())
    D.show()
    D.remove_plot(TEBoxPlot)


def plots2():

    D = Dashboard()  # optionally we can specify the verbosity of the output messages. default is logger.INFO

    # D.list_runs()

    D.add_series('Point_Slam_filtered',
                 tour_names='N_tour',
                 localization_technique='pointslam',
                 filter_status='true',
                 class_method='ground_truth')

    D.add_series('gmapping_filtered',
                 tour_names='N_tour',
                 localization_technique='gmapping',
                 filter_status='true',
                 class_method='ground_truth')

    D.add_series('amcl_filtered',
                 tour_names='N_tour',
                 localization_technique='amcl',
                 filter_status='true',
                 class_method='ground_truth')

    D.add_plot(TranslationError(True))
    D.show()
    D.remove_plot(TranslationError)

    D.add_plot(TEBoxPlot())
    D.show()
    D.remove_plot(TEBoxPlot)

    D.add_plot(TrajectoryPlot(only_gt=True))
    D.show()
    D.remove_plot(TrajectoryPlot)


def single_comp(tour_names='A_tour', scenarios='Sc1_few_wanderers', method='pointslam'):
    D = Dashboard()  # optionally we can specify the verbosity of the output messages. default is logger.INFO

    D.list_runs()

    D.add_series(tour_names + '_' + scenarios[:3] + '_raw',
                 tour_names=tour_names,
                 scenarios=scenarios,
                 localization_technique=method,
                 filter_status='false')

    D.add_series(tour_names + '_' + scenarios[:3] + '_flt',
                 tour_names=tour_names,
                 scenarios=scenarios,
                 localization_technique=method,
                 filter_status='true',
                 class_method='ground_truth')

    # D.add_plot(TEBoxPlot())
    # D.show()
    # D.remove_plot(TEBoxPlot)

    D.add_plot(TrajectoryPlot(only_gt=False))
    D.show()
    D.remove_plot(TrajectoryPlot)


def clear_failed_tours():

    D = Dashboard()  # optionally we can specify the verbosity of the output messages. default is logger.INFO

    D.list_runs()

    #D.add_series('Point_Slam_filtered', localization_technique='pointslam', filter_status='true')

    D.add_series('failed',
                 # tour_names='C_tour',
                 success_status='false')

    D.remove_series_dirs('failed')

    print('\n\nRemoving tours\n')

    D.list_runs()


def ICRA_2021_Exp1():
    """
    Here go for the first an main Experiment plots
    """

    D = Dashboard()  # optionally we can specify the verbosity of the output messages. default is logger.INFO
    D.list_runs(100)

    # ---------------------------------------------------------------------------------------------
    # Set round dates

    # Round 1
    round_D1 = ['2020-10-12-22-06-54']
    round_D2 = ['2020-10-13-00-23-56']
    D1_Sc4 = ['2020-10-22-20-57-12']
    D2_Sc4 = ['2020-10-22-21-05-26']

    # Round 2
    round_D1 += ['2020-10-16-12-29-11']
    round_D2 += ['2020-10-16-14-56-40']
    D1_Sc4 += ['2020-10-22-01-25-33']
    D2_Sc4 += ['2020-10-22-19-29-41']

    # Round 3
    round_D1 += ['2020-10-19-17-25-50']
    round_D2 += ['2020-10-19-20-23-25']
    D1_Sc4 += ['2020-10-22-21-17-35']
    D2_Sc4 += ['2020-10-22-22-04-14']

    # Round 4
    round_D1 += ['2020-10-28-16-44-33']
    round_D2 += ['2020-10-29-01-01-19']
    D1_Sc4 += ['2020-10-28-13-49-46']
    D2_Sc4 += ['2020-10-28-14-19-38']
    
    
    # Round GT
    round_D1 += ['2020-10-20-13-55-57']
    round_D2 += ['2020-10-20-17-01-25']
    D1_Sc4 += ['2020-10-22-23-26-57']
    D2_Sc4 += ['2020-10-23-01-18-39']

    
    
    # ---------------------------------------------------------------------------------------------
    # Get simulation data

    n_rounds = len(round_D1) - 1
    n_scen = 4
    tour = ''
    series_names = []
    series_TE = []
    series_YE = []
    series_success_rate = []
    series_success = []

    username = os.environ['USER']
    log_path = '/home/' + username + '/Myhal_Simulation/simulated_runs'

    for d, (D1, D2) in enumerate(zip(round_D1, round_D2)):

        # Get log names in the chosen dates
        log_names = [f for f in listdir(log_path) if (f <= D2 and f >= D1)]
        log_names += [f for f in listdir(log_path) if (f <= D2_Sc4[d] and f >= D1_Sc4[d])]
        log_names = np.sort(log_names)

        # Get data
        log_gt_trajs = []
        log_loc_trajs = []
        for log_name in log_names:

            fast_data_file = os.path.join(
                log_path, log_name, 'fast_data.pickle')
            if os.path.exists(fast_data_file):
                with open(fast_data_file, 'rb') as f:
                    gt_traj, loc_traj = pickle.load(f)

            else:

                runpath = os.path.join(log_path, log_name, 'logs-' + log_name)
                with open(os.path.join(runpath, 'processed_data.pickle')) as data_f:
                    data_d = pickle.load(data_f)

                gt_traj = data_d['gt_traj']
                loc_traj = data_d['amcl_traj'] if (
                    'amcl_traj' in data_d.keys()) else data_d['gmapping_traj']

                with open(fast_data_file, 'wb') as f:
                    pickle.dump((gt_traj, loc_traj), f)

            log_gt_trajs.append(gt_traj)
            log_loc_trajs.append(loc_traj)

        # Regroup by scenario
        for scenario in ['Sc1_few_wanderers_some_tables', 
                         'Sc2_crowd_wanderers_some_tables',
                         'Sc3_some_followers_some_tables',
                         'Sc4_some_followers_more_tables']:

            name = 'R{:d}_{:s}'.format((d % (n_rounds + 1)) + 1, scenario[:3])
            series_names.append(name)

            s_dist = []
            a_dist = []
            success_rates = []
            successes = []
            for log_i, log_name in enumerate(log_names):
                if scenario in D.handler.run_map[log_name].meta['scenarios']:
                    error = pu.translation_error(log_loc_trajs[log_i], log_gt_trajs[log_i])
                    # loc_traj_2D = np.vstack((log_loc_trajs[log_i]['pos_x'], log_loc_trajs[log_i]['pos_y']))
                    # gt_traj_2D = np.vstack((log_gt_trajs[log_i]['pos_x'], log_gt_trajs[log_i]['pos_y']))
                    # error0 = np.linalg.norm(loc_traj_2D - gt_traj_2D, axis=0)
                    # print(np.max(np.abs(error - error0)))

                    s_dist += list(error)
                    a_dist += list(pu.yaw_error(log_loc_trajs[log_i], log_gt_trajs[log_i]))
                    success_rates.append(D.handler.run_map[log_name].meta['targets_reached'])
                    successes.append(D.handler.run_map[log_name].meta['success_status'])

            series_TE.append(np.array(s_dist))
            series_YE.append(np.array(a_dist))
            if success_rates:
                series_success_rate.append(np.mean(success_rates))
                series_success.append(np.prod(successes))
            else:
                series_success_rate.append(None)
                series_success.append(None)

    series_names_to_inds = {name: i for i, name in enumerate(series_names)}


    # ---------------------------------------------------------------------------------------------
    # Show Success table
    plot_success = True
    if plot_success:

        print_str = '\n\n          '
        for scenario_i in range(n_scen):
            print_str += ' Sc{:d}   '.format(scenario_i+1)
        print_str += 'Avg  \n'

        for round_i in range(n_rounds):

            print_str += 'Round {:d}   '.format(round_i+1)

            all_rates = []
            for scenario_i in range(n_scen):
                name = 'R{:d}_Sc{:d}'.format(round_i+1, scenario_i+1)
                if name not in series_names_to_inds.keys():
                    print_str += ' --    '
                else:
                    series_i = series_names_to_inds[name]
                    rate = series_success_rate[series_i]
                    if rate is None:
                        print_str += ' --    '
                    else:
                        print_str += '{:3d}%   '.format(int(rate * 100))
                        all_rates.append(rate)
                
                #series_success[series_i]

            if all_rates:
                print_str += '{:3d}% \n'.format(int(np.mean(all_rates) * 100))
            else:
                print_str += ' --  \n'

        print_str += '\n\n'
        print(print_str)

    # ---------------------------------------------------------------------------------------------
    # Collect data for plots
    
    # Color for each scenario
    sc_col = ['limegreen', 'darkorange', 'firebrick', 'dimgrey']

    y_data = [[]]
    y_data_yaw = [[]]
    x_data = ['']
    colors = ['black']

    for round_i in range(n_rounds + 1):
        for scenario_i in range(n_scen + 2):

            if scenario_i < n_scen:
                name = 'R{:d}_Sc{:d}'.format(round_i+1, scenario_i+1)
                if name not in series_names_to_inds.keys():
                    continue
                series_i = series_names_to_inds[name]
                y_data.append(series_TE[series_i] * 100)
                y_data_yaw.append(series_YE[series_i] * 180 / np.pi)
                colors.append(sc_col[scenario_i])
                if scenario_i == 1:
                    if round_i == n_rounds:
                        x_data.append('    Round GT')
                    else:
                        x_data.append('     Round {:d}'.format(round_i+1))
                else:
                    x_data.append('')
            else:
                if round_i >= n_rounds:
                    break
                x_data.append('')
                y_data.append([])
                y_data_yaw.append([])
                colors.append('black')

    x_data.append('')
    y_data.append([])
    y_data_yaw.append([])
    colors.append('black')
    

    # ---------------------------------------------------------------------------------------------
    # Plot TE

    fig, ax = plt.subplots(figsize=(6.5, 3.4))
    fig.subplots_adjust(left=0.11, right=0.95, top=0.95, bottom=0.11)

    # Add a horizontal grid to the plot, but make it very light in color
    # so we can use it for reading data values but not be distracting
    ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
                  alpha=0.5)

    # Hide these grid behind plot objects
    ax.set_axisbelow(True)
    ax.set_ylabel('Translation Error (cm)')

    bp = ax.boxplot(y_data, labels=x_data, patch_artist=True,
                    showfliers=False, widths=0.8)
    #plt.setp(bp['whiskers'], color='blue')
    plt.setp(bp['medians'], color='k')
    
    # Dashed line for GT
    plt.setp(bp['boxes'][-6:], ls='--')
    plt.setp(bp['whiskers'][-12:], ls='--')

    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)

    plt.legend(bp['boxes'][1:5], ('Easy', 'Medium', 'Hard', 'Extreme'), 
               title='Scenarios',
               fontsize='small')

    plt.savefig('Exp1.pdf')
    plt.show()


    # ---------------------------------------------------------------------------------------------
    # Plot YE

    plot_yaw = False
    if plot_yaw:
        fig, ax = plt.subplots(figsize=(6.5, 4))
        fig.subplots_adjust(left=0.11, right=0.95, top=0.95, bottom=0.11)

        # Add a horizontal grid to the plot, but make it very light in color
        # so we can use it for reading data values but not be distracting
        ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
                    alpha=0.5)

        # Hide these grid behind plot objects
        ax.set_axisbelow(True)
        ax.set_ylabel('Translation Error (degrees)')

        bp = ax.boxplot(y_data_yaw, labels=x_data, patch_artist=True,
                        showfliers=False, widths=0.8)
        #plt.setp(bp['whiskers'], color='blue')
        plt.setp(bp['medians'], color='k')

        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)

        plt.savefig('Exp1_yaw.pdf')
        plt.show()









    return


def ICRA_2021_Exp2():
    """
    Here go for the first an main Experiment plots
    """

    D = Dashboard()  # optionally we can specify the verbosity of the output messages. default is logger.INFO

    D.list_runs(100)

    # Round 3
    round_D1 = ['2020-10-19-17-25-50']
    round_D2 = ['2020-10-19-20-23-25']

    # Round GT
    round_D1 += ['2020-10-20-13-55-57']
    round_D2 += ['2020-10-20-17-01-25']

    tour = ''
    series_names = []
    for d, (D1, D2) in enumerate(zip(round_D1, round_D2)):
        for scenario in ['Sc1_few_wanderers_some_tables', 'Sc2_crowd_wanderers_some_tables', 'Sc3_some_followers_some_tables']:

            if scenario:
                name = 'Round{:d}_'.format(d+1) + scenario[:3]
                series_names.append(name)
                D.add_series(name,
                             earliest_date=D1,
                             latest_date=D2,
                             tour_names=tour,
                             scenarios=scenario)

    # Color for each scenario
    sc_col = ['limegreen', 'darkorange', 'firebrick']
    round_names = ['Round3', 'GT']

    y_data = [[]]
    x_data = ['']
    colors = ['black']
    for round_i in range(2):
        for scenario_i in range(5):
            if scenario_i < 3:
                name = 'Round{:d}_Sc{:d}'.format(round_i+1, scenario_i+1)
                series = D.display.series_map[name]
                s_dist = []
                for run in series.runs:
                    gt_traj = run.get_data('gt_traj')
                    loc_traj = run.get_data('amcl_traj') if (
                        'amcl_traj' in run.keys()) else run.get_data('gmapping_traj')
                    error = pu.translation_error(loc_traj, gt_traj)
                    s_dist += list(error * 1000)
                y_data.append(s_dist)
                colors.append(sc_col[scenario_i])

                if scenario_i == 1:
                    x_data.append('Sc{:d}\n{:s}'.format(
                        scenario_i+1, round_names[round_i]))
                else:
                    x_data.append('Sc{:d}'.format(scenario_i+1))
            else:
                if round_i > 0:
                    break
                x_data.append('')
                y_data.append([])
                colors.append('black')

    x_data.append('')
    y_data.append([])
    colors.append('black')

    fig, ax = plt.subplots(figsize=(6.5, 4))
    fig.subplots_adjust(left=0.11, right=0.95, top=0.95, bottom=0.11)

    # Add a horizontal grid to the plot, but make it very light in color
    # so we can use it for reading data values but not be distracting
    ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
                  alpha=0.5)

    # Hide these grid behind plot objects
    ax.set_axisbelow(True)
    ax.set_ylabel('Translation Error (mm)')

    bp = ax.boxplot(y_data, labels=x_data, patch_artist=True,
                    showfliers=False, widths=0.8)
    #plt.setp(bp['whiskers'], color='blue')
    plt.setp(bp['medians'], color='k')

    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)

    plt.savefig('Exp2.pdf')
    plt.show()

    return


def ICRA_2021_Exp3():
    """
    Here go for the first an main Experiment plots
    """

    D = Dashboard()  # optionally we can specify the verbosity of the output messages. default is logger.INFO

    D.list_runs(100)


    # AMCL Round 1
    method = ['AMCL']
    round_D1 = ['2020-10-29-06-07-47']
    round_D2 = ['2020-10-29-07-06-45']
    D1_Sc4 = ['2020-10-29-18-31-17']
    D2_Sc4 = ['2020-10-29-18-31-17']

    # AMCL Round 2
    method += ['AMCL']
    round_D1 += ['2020-10-29-07-15-44']
    round_D2 += ['2020-10-29-07-56-29']
    D1_Sc4 += ['2020-10-29-14-34-08']
    D2_Sc4 += ['2020-10-29-14-46-19']
    
    # Gmapping Round 1
    method += ['Gmapping']
    round_D1 += ['2020-10-29-04-11-00']
    round_D2 += ['2020-10-29-04-54-32']
    D1_Sc4 += ['']
    D2_Sc4 += ['']

    # Gmapping Round 2
    method += ['Gmapping']
    round_D1 += ['2020-10-29-05-03-57']
    round_D2 += ['2020-10-29-05-47-42']
    D1_Sc4 += ['']
    D2_Sc4 += ['']

    # PointMap Round 1
    method += ['PointMap*']
    round_D1 += ['2020-10-29-08-14-18']
    round_D2 += ['2020-10-29-09-01-19']
    D1_Sc4 += ['']
    D2_Sc4 += ['']

    # PointMap Round 2
    method += ['PointMap*']
    round_D1 += ['2020-10-29-09-12-09']
    round_D2 += ['2020-10-29-09-52-16']
    D1_Sc4 += ['']
    D2_Sc4 += ['']

    n_rounds = 2
    n_scen = 2
    series_names = []
    series_TE = []

    username = os.environ['USER']
    log_path = '/home/' + username + '/Myhal_Simulation/simulated_runs'

    for d, (D1, D2) in enumerate(zip(round_D1, round_D2)):

        # Get log names in the chosen dates
        log_names = [f for f in listdir(log_path) if (f <= D2 and f >= D1)]
        log_names += [f for f in listdir(log_path) if (f <= D2_Sc4[d] and f >= D1_Sc4[d])]
        log_names = np.sort(log_names)

        # Get data
        log_gt_trajs = []
        log_loc_trajs = []
        for log_name in log_names:

            fast_data_file = os.path.join(
                log_path, log_name, 'fast_data.pickle')
            if os.path.exists(fast_data_file):
                with open(fast_data_file, 'rb') as f:
                    gt_traj, loc_traj = pickle.load(f)

            else:

                runpath = os.path.join(log_path, log_name, 'logs-' + log_name)
                with open(os.path.join(runpath, 'processed_data.pickle')) as data_f:
                    data_d = pickle.load(data_f)

                gt_traj = data_d['gt_traj']
                loc_traj = data_d['amcl_traj'] if (
                    'amcl_traj' in data_d.keys()) else data_d['gmapping_traj']

                with open(fast_data_file, 'wb') as f:
                    pickle.dump((gt_traj, loc_traj), f)

            log_gt_trajs.append(gt_traj)
            log_loc_trajs.append(loc_traj)

        # Regroup by scenario
        for scenario in ['Sc2_crowd_wanderers_some_tables', 'Sc3_some_followers_some_tables']:

            name = '{:s}_R{:d}_{:s}'.format(
                method[d], (d % n_rounds) + 1, scenario[:3])
            series_names.append(name)

            s_dist = []
            for log_i, log_name in enumerate(log_names):
                if scenario in D.handler.run_map[log_name].meta['scenarios']:
                    error = pu.translation_error(
                        log_loc_trajs[log_i], log_gt_trajs[log_i])
                    # loc_traj_2D = np.vstack((log_loc_trajs[log_i]['pos_x'], log_loc_trajs[log_i]['pos_y']))
                    # gt_traj_2D = np.vstack((log_gt_trajs[log_i]['pos_x'], log_gt_trajs[log_i]['pos_y']))
                    # error0 = np.linalg.norm(loc_traj_2D - gt_traj_2D, axis=0)
                    # print(np.max(np.abs(error - error0)))
                    s_dist += list(error)

            series_TE.append(np.array(s_dist))

    series_names_to_inds = {name: i for i, name in enumerate(series_names)}

    # Color for each scenario
    sc_col = ['darkorange', 'firebrick']

    fig, axs = plt.subplots(1, 3, figsize=(8, 3.2), sharex=True, sharey=False)
    fig.subplots_adjust(left=0.11, right=0.95, top=0.95, bottom=0.11)

    axs = list(axs.ravel())

    print('\n-----------------------\n')

    for ax_i, loc_meth in enumerate(['AMCL', 'Gmapping', 'PointMap*']):

        y_data = [[]]
        x_data = ['']
        colors = ['black']
        for round_i in range(n_rounds):
            for scenario_i in range(n_scen + 2):
                if scenario_i < n_scen:
                    name = '{:s}_R{:d}_Sc{:d}'.format(
                        loc_meth, round_i+1, scenario_i+2)
                    
                    if name not in series_names_to_inds.keys():
                        continue
                    series_i = series_names_to_inds[name]
                    if 'PointMap' in loc_meth:
                        y_data.append(series_TE[series_i] * 100)
                    else:
                        y_data.append(series_TE[series_i] * 100)
                    colors.append(sc_col[scenario_i])
                    if scenario_i == 0:
                        if round_i == 0:
                            x_data.append('     No prediction')
                        else:
                            x_data.append('      Network 3')
                    else:
                        x_data.append('')
                else:
                    if round_i > 0:
                        break
                    x_data.append('')
                    y_data.append([])
                    colors.append('black')
                

        x_data.append('')
        y_data.append([])
        colors.append('black')

        # Add a horizontal grid to the plot, but make it very light in color
        # so we can use it for reading data values but not be distracting
        axs[ax_i].yaxis.grid(True, linestyle='-',
                             which='major', color='lightgrey', alpha=0.5)

        # labels
        if ax_i == 0:
            axs[ax_i].set_ylabel('Translation Error (cm)')
        if ax_i in [0, 1]:
            axs[ax_i].set_ylim(bottom=-99/20, top=99)
        if ax_i == 2:
            axs[ax_i].set_ylim(bottom=-4.9/20, top=4.9)

        # Title
        # axs[ax_i].set_title(loc_meth)
        axs[ax_i].text(0.8, .93, loc_meth,
                       transform=axs[ax_i].get_xaxis_transform(),
                       horizontalalignment='left', fontsize='medium',
                       weight='roman',
                       color='k')

        # Hide the grid behind plot objects
        axs[ax_i].set_axisbelow(True)

        bp = axs[ax_i].boxplot(y_data, labels=x_data,
                               patch_artist=True, showfliers=False, widths=0.8)
        plt.setp(bp['medians'], color='k')

        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)



           
        if ax_i == 2:
            plt.legend(bp['boxes'][1:3], ('Medium', 'Hard'), 
                    title='Scenarios',
                    fontsize='small')

    plt.savefig('Exp3.pdf')
    plt.show()

    return


def tmp():

    D = Dashboard()  # optionally we can specify the verbosity of the output messages. default is logger.INFO

    D.list_runs(30)

    #D.plot_run('2020-09-29-02-05-05', TrajectoryPlot(only_gt=False))

    #D.plot_run('2020-09-30-16-08-38', TrajectoryPlot(only_gt=False))
    #D.plot_run('2020-09-30-16-08-38', TranslationError(True))

    #D.plot_run('2020-10-01-14-12-22', TrajectoryPlot(only_gt=False))

    D.plot_run('2020-10-23-12-23-57', TranslationError(True))
    D.plot_run('2020-10-23-12-23-57', TEBoxPlot(True))
    D.plot_run('2020-10-20-18-18-06', TranslationError(True))
    D.plot_run('2020-10-20-18-18-06', TEBoxPlot(True))

    a = 1/0


if __name__ == '__main__':

    # clear_failed_tours()
    # a = 1/0

    tmp()
    a = 1/0

    # plots1()

    # plots2()

    #ICRA_2021_Exp1()

    # ICRA_2021_Exp2()

    ICRA_2021_Exp3()

    # single_comp(tour_names='C_tour',
    #             scenarios='Sc1_few_wanderers')

    # single_comp(tour_names='B_tour',
    #             scenarios='Sc2_crowd_wanderers_some_tables')

    # single_comp(tour_names='B_tour',
    #             scenarios='Sc3_some_followers_some_tables')
