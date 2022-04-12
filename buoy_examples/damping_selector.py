# Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pysofar.sofar import SofarApi
from pysofar import get_token
from mhkit.wave.resource import energy_flux
from mhkit.wave.resource import energy_period
from mhkit.wave.resource import significant_wave_height
from mhkit.wave.resource import wave_number
import pandas as pd
import xarray as xr
import numpy as np
from scipy.stats import qmc
from datetime import datetime
import os
import matplotlib.pyplot as plt
import warnings
from scipy.interpolate import interp1d


def get_gain(buoy_id:str='SPOT-0589',
             sofar_token:str='d3237841f268baa5d902879e9013de',
             schedule_file_name:str='damping_schedule',
             gain_default_range:tuple=(1.1, 1.2)) -> float:
    """Get latest conditions from Spotter and find gain based on:

        (a) has not yet been run and
        (b) is closest to the conditions observed by the Spotter buoy, giving a
        damping factor to be set on the WEC.

    Parameters
    ----------
    buoy_id : str, optional
        identification string for desired spotter buoy.
            SPOT-0589: Sandia's buoy
            SPOT-0222: Sofar's public buoy
    sofar_token : str, optional
        generated by loging into Sofar website; gives access to all buoys
        registered under your username,
        by default 'd3237841f268baa5d902879e9013de'
    schedule_file_name : str, optional
        file created by generate_damping_schedule.py,
        by default 'damping_schedule'
    gain range to use when Spotter does not respond  : tuple, optional
        _description_, by default (1.1, 1.2)

    Returns
    -------
    float
        gain factor
    """

    query_time = datetime.now()
    script_path = os.path.dirname(os.path.realpath(__file__))

    rho = 1025      # density [kg/m^3]
    h = 100         # depth [m]
    g = 9.81        # gravity [m/s^2]


    try:
        ds = xr.open_dataset(os.path.join(script_path,
                                        schedule_file_name + '.nc'))
        ds.close()
    except Exception as e:
        print(f"Schedule file ('f{schedule_file_name}') not found")
        print(e)

    hist_schedule_file_name = os.path.join(script_path,
                                        schedule_file_name + '_history.csv')
    if os.path.exists(hist_schedule_file_name):
        hist_ds = pd.read_csv(hist_schedule_file_name).to_xarray()
        been_run = hist_ds.index.dropna('index')
        failed_run_num = np.atleast_1d(hist_ds.index.isnull().sum())[0]

    else:
        been_run = np.empty(0)
        failed_run_num = 0

    total_num = len(ds.index)
    been_run_num = len(been_run)
    remaining_num = total_num - been_run_num

    b = np.zeros_like(ds.index)
    if been_run_num > 0: b[been_run.astype('int').values.tolist()] = True
    ds = xr.merge([ds, xr.DataArray(b.astype('bool'),
                                    dims='index',
                                    coords=dict(index=ds.index.values),
                                    name='been_run')])

    print('')
    print(query_time)
    print(f"{'Schedule cases run to-date: ':<35}" + f"{been_run_num}")
    print(f"{'Schedule cases remaining: ':<35}" + f"{remaining_num}")
    print(f"{'Non-schedule cases run to-date: ':<35}" + f"{failed_run_num}")



    try:
        # Get latest observations from Spotter buoy
        get_token()
        sofar = SofarApi(sofar_token)
        spotter_grid = sofar.get_spotters()
        ind = [spotter.id for spotter in spotter_grid].index(buoy_id)
        spotter = spotter_grid[ind]
        print(f"{'Successfull connected to: ':<35}" + f"{spotter.id}")
        dat = spotter.latest_data()['frequency']
        buoy_time = pd.to_datetime(dat['timestamp']).to_pydatetime()
        print(f"{'Buoy timestamp: ':<35}" + f"{buoy_time}")
        print(f"{'Location (lat, lon): ':<35}"
              + (f"{[dat['latitude'], dat['longitude']]}"))

        # Calculate spectral quantities
        S = pd.Series(index=dat['frequency'], data=dat['varianceDensity'])

        Te = energy_period(S=S,
                    frequency_bins=np.array(dat['df'])).values[0][0]

        Hm0 = significant_wave_height(S=S,
                    frequency_bins=np.array(dat['df'])).values[0][0]

        J_kW = energy_flux(S=S,
                        h=h,
                        rho=rho,
                        g=g).values[0][0]/1e3

        k = wave_number(1/Te, 100).values.squeeze().item()
        wave_length = 2*np.pi/k
        Pmax_kW = J_kW / k

        # Find nearest case (which has not yet been run)
        obs = np.array([Hm0, Te])
        a = xr.merge([ds.Hm0, ds.Te]).to_array()
        b = xr.DataArray(data=obs, coords=dict(variable=['Hm0','Te']))
        c = a - b
        norms = xr.DataArray(data=np.linalg.norm(c.to_numpy(),axis=0),
                            coords=dict(index=range(c.values.shape[1])),
                            name='norm')
        ds = xr.merge([ds, norms])
        idx = ds.where(~ds.been_run).norm.argmin(skipna=True).values.item()
        dist = norms[idx].values.item()
        closest = np.array([ds.isel(index=idx).Hm0.values.item(),
                            ds.isel(index=idx).Te.values.item()])

        gain_factor = ds.gain_factor[idx].values.item()
    except Exception as e:
        print('Failed to connect with Spotter buoy')
        print(e)
        buoy_time = np.nan
        obs = np.array([np.nan, np.nan])
        idx = np.nan
        JkW = np.nan
        closest = np.array([np.nan, np.nan])
        gain_factor = qmc.scale(np.atleast_2d(np.random.rand(1)),
                                gain_default_range[0],
                                gain_default_range[1]).squeeze()


    print(f"{'Latest spotter obs. (Hm0, Te): ':<35}" + f"{obs}")
    print(f"{'Nearest unrun case (Hm0, Te): ':<35}" + f"{closest}")
    print(f"{'Gain factor: ':<35}" + f"{gain_factor}" + f" (index: {idx})")
    print(f"{'Incident wave power [kW/m]: ':<35}" + f"{J_kW:.2f}")


    plt.close('all')
    fig, ax = plt.subplots()

    ax.scatter(obs[1], obs[0],
                label='Observed')
    ax.scatter(closest[1],closest[0],
                label='Closest')
    ax.scatter(ds.where(~ds.been_run).Te,
            ds.where(~ds.been_run).Hm0,
                marker='.',
            s=5,
            c='k',
            label='Unrun cases')
    ax.scatter(ds.where(ds.been_run).Te,
            ds.where(ds.been_run).Hm0,
                marker='x',
                s=15,
                c='k',
                label='Run cases')

    ax.legend(loc=2,
            bbox_to_anchor=(0,1.1),
            ncol=4)

    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.set_xlabel(ds.Te.long_name)
    ax.set_ylabel(ds.Hm0.long_name)

    fig.savefig(schedule_file_name + '_latest.pdf')

    # Update the CSV file

    p = pd.DataFrame(data=np.atleast_2d(np.array([query_time,
                                                  buoy_time,
                                                obs[0],
                                                obs[1],
                                                closest[0],
                                                closest[1],
                                                J_kW,
                                                idx,
                                                gain_factor
                                                ])),
                    columns=['query_time',
                             'buoy_time',
                            'Hm0_obs',
                            'Te_obs',
                            'J_obs',
                            'Hm0_closest',
                            'Te_closest',
                            'index',
                            'gain'
                            ])


    hist_file_exist = os.path.exists(hist_schedule_file_name)
    if hist_file_exist:
        mode = 'a'
    else:
        mode = 'w'

    p.to_csv(hist_schedule_file_name,
            mode=mode,
            header=not hist_file_exist,
            index=False)

    return gain_factor

def generate_schedule(schedule_file_name:str='damping_schedule') -> None:
    """Generate schedule of gains (call this first and only once)

    Parameters
    ----------
    schedule_file_name : str, optional
        file created by generate_damping_schedule.py,
        by default 'damping_schedule'
    """

    # These can be used to keep the gain greater than a certain level if Hm0 is
    # greater than a specified threshold. The format is {Hm0: gain}, where
    # gain_selected > gain if Hm0_observed > Hm0
    gain_thresholds = {
        2: 0.9,
        3: 1,
        }

    variables = ['Hm0','Te','gain_factor']
    lower_bounds = [0, 0, 0.7]
    upper_bounds = [8, 16, 1.2]
    n_exp = int(2e3)

    # Generate samples
    sampler = qmc.LatinHypercube(d=3,       # number of dimensions
                                seed=1,     # get the same set of results each time
                                )
    sample = sampler.random(n=n_exp)        # take LHS samples



    # map Hm0 to 0:1
    x = [0, 8]
    y = [0, 1]
    f = interp1d(x, y,
                bounds_error=False,
                fill_value=(lower_bounds[0], upper_bounds[0]),
                )

    # scale LHS samples for each gain threshold regime
    gain_thresholds[0] = lower_bounds[2]
    gtks = sorted(gain_thresholds.keys())
    ss_list = []
    for Hm0_thresh_lower, Hm0_thresh_upper in zip(gtks, gtks[1:] + [np.Inf]):
        gain_thresh = gain_thresholds[Hm0_thresh_lower]
        lower_bounds_1 = lower_bounds[:2] + [gain_thresholds[Hm0_thresh_lower]]
        print('{:} <= Hm0 <= {:<4}: {:<3} <= gain <= {:}'.format(Hm0_thresh_lower,
                                                        Hm0_thresh_upper,
                                                        lower_bounds_1[2],
                                                        upper_bounds[2]))

        sub_sample = sample[np.where(np.logical_and(sample[:,0]>=f(Hm0_thresh_lower),
                                                    sample[:,0]<=f(Hm0_thresh_upper)))]
        print(sub_sample.shape)
        sub_sample_scaled = qmc.scale(sub_sample, lower_bounds_1, upper_bounds)
        ss_list += [sub_sample_scaled]

    sample_scaled = np.concatenate(ss_list)


    # Save results
    ds = pd.DataFrame(data=sample_scaled,
                    columns=variables).to_xarray()

    ds['Te'].attrs['units'] = 's'
    ds['Te'].attrs['long_name'] = 'Energy period'

    ds['Hm0'].attrs['units'] = 'm'
    ds['Hm0'].attrs['long_name'] = 'Significant wave height'

    ds['gain_factor'].attrs['units'] = ' '
    ds['gain_factor'].attrs['long_name'] = 'Gain factor'

    fig,ax = plt.subplots()
    ds.plot.scatter(x='Te', y='Hm0', hue='gain_factor', alpha=0.5, ax=ax)
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    fig.savefig(schedule_file_name + '.pdf')

    if not os.path.exists(schedule_file_name + '.nc'):
        ds.to_netcdf(schedule_file_name + '.nc')
    else:
        warnings.warn("'{}' already exists, will not overwrite".format(schedule_file_name + '.nc'))
