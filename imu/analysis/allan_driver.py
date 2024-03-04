import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import allantools 


"""
I converted the .bag file using rostopic echo /vectornav -b /home/rohit/.ros/Clocation.bag -p > file.csv, 
then stripped the gyro x,y,z using .strip('*') and stored it into gdata.csv
I have uploaded the logics in readcsv.py and readcsv.py
"""

def allanvariance(gx,Fs,nb=200):
    t0 = 1/Fs
    theta = np.cumsum(gx) * t0 
    L = len(theta)
    maxM = 2**np.floor(np.log2(L/2))
    maxNumM = nb
    m = np.ceil(np.logspace(np.log10(1), np.log10(maxM), maxNumM))#.astype(int)
    m = np.unique(m)  # Remove duplicates
    tau = m * t0

    avar = np.zeros(len(m))
    for i, mi in enumerate(m):
        differ = (theta[int(2*mi):L] - (2*theta[int(mi):L-int(mi)]) + theta[:L-int(2*mi)])
        nanIndices = np.isnan(differ)
        differ[nanIndices] = 0.00
        avar[i] = np.sum(differ**2)

    avar = avar / (2 * tau**2 * (L - 2*m))
    adev = np.sqrt(avar)
    return [adev,tau]




def anglerandwalk(tau,adev):
    slope = -0.5
    logtau = np.log10(tau)
    logadev = np.log10(adev)
    dlogadev = np.diff(logadev) / np.diff(logtau)
    i = np.argmin(np.abs(dlogadev - slope))
    b = logadev[i] - slope*logtau[i]
    logN = slope*np.log10(1) + b
    N = 10**logN

    # Plot Angle Random Walk
    tauN = 1
    lineN = N / np.sqrt(tau)

    return [lineN,tauN, N]
    


# Bias Instability
def biasinstability(tau,adev):
    slope = 0
    logtau = np.log10(tau)
    logadev = np.log10(adev)
    dlogadev = np.diff(logadev) / np.diff(logtau)
    i = np.argmin(np.abs(dlogadev - slope))
    b = logadev[i] - slope*logtau[i]
    scfB = np.sqrt(2*np.log(2)/np.pi)
    logB = b - np.log10(scfB)
    B = 10**logB

    # Plot Bias Instability
    tauB = tau[i]
    lineB = B * scfB * np.ones_like(tau)

    return [lineB,tauB, scfB*B]


def raterandmwalk(tau,adev):

    slope = 0.5
    logtau = np.log10(tau)
    logadev = np.log10(adev)
    dlogadev = np.diff(logadev) / np.diff(logtau)
    i = np.argmin(np.abs(dlogadev - slope))
    b = logadev[i] - slope*logtau[i]
    logK = slope*np.log10(3) + b
    lineK = (10**logK)* np.sqrt(tau/3)

    idx = np.argwhere(np.diff(np.sign(adev - lineK))).flatten()                     
    nonzero_idxs = np.where(adev[idx] != 0)[0]

    min_nonzero_idx = idx[nonzero_idxs[np.argmin(adev[idx[nonzero_idxs]])]]
    tauk=tau[min_nonzero_idx]
    k=adev[min_nonzero_idx]


    return [lineK,tauk, k]



    
if __name__ == "__main__":
    
    data = pd.read_csv('gdata.csv')
    Fs = 40
    

    for i,a in [[0,'X'], [1,'Y'],[2,'Z']]:
        gx = data.iloc[:, i].values #rad/s
        Max=len(gx)
        # Max=20000

        print(f'Data corresponds to axis: {a}')
        



        [ad,tau]=allanvariance(gx,Fs,nb=Max)
        [lineB,tauB, B]=biasinstability(tau,ad)
        [lineN,tauN,N]=anglerandwalk(tau,ad)
        [lineK,tauk, K]=raterandmwalk(tau,ad)




        # plt.figure()
        plt.figure(figsize=(10, 6))  # Adjust the width and height as needed
        plt.loglog(tau, ad,color='r',label=a+'-axis (rad/s)')

        plt.loglog(tau, lineB, '--',color='b',label='slope at Bias Instability(BI)')
        plt.scatter(tauB, B, color='b', label=f'BI = {B:.8f} (rad/s) '+a+'-axis')
        plt.text(tauB, B, 'B')


        plt.loglog(tau, lineN, '--',color='g',label='slope at Angle Random Walk(ARW) ('+r'$\tau$'+'=1s )')
        plt.scatter(tauN, N,color='g',label=f'ARW = {N:.8f} (rad/s)($\sqrt{{\mathrm{{Hz}}}})$ '+a+'-axis')
        plt.text(tauN, N, 'N')
        plt.axhline(y=N, color='y', linestyle='--')
        plt.axvline(x=1, color='y', linestyle='--')



        plt.loglog(tau, lineK, '--',color='orange',label='slope at Rate Random Walk(RRW)')
        plt.scatter(tauk ,K , color='orange', label=f'RRW = {K:.8f} (rad/s)($\sqrt{{\mathrm{{Hz}}}})$ '+a+'-axis')
        plt.text(tauk, K, 'K')


        plt.title(a+'-axis Gyro Allan Deviation: Noise Parameters')
        plt.xlabel(r'$\tau$ [sec]')
        plt.ylabel('Deviation [rad/sec]')
        plt.legend(fontsize='small')



        plt.xscale('log')
        plt.yscale('log')
        plt.grid(True, which="both", ls="-", color='0.65')
        

        # Your plotting code here

        plt.savefig(a+'-axis_Gyro_Allan_Deviation_Noise_parameters.png', dpi=900)
        plt.show()