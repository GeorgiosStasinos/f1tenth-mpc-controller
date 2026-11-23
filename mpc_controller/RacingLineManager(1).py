"""Experimental racing-line optimizer: actual curvature SQP with QP subproblems.

Designed for SaoPaulo_centerline.csv and similarly sampled, closed centerlines.
Default ey_margin=0.0, no extra apex offset restriction. The model optimizes a
REFERENCE POINT only: zero margin is not a physical collision-clearance guarantee.

Usage:
  python RacingLineManager_curvature_SQP.py /path/to/SaoPaulo_centerline.csv --iterations 25

Requires numpy, scipy, pandas, matplotlib; OSQP optional (recommended).
"""
from __future__ import annotations
import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from scipy import sparse
from scipy.optimize import minimize

try:
    import osqp
except ImportError:
    osqp=None


def load_track(path):
    import pandas as pd
    a = pd.read_csv(path)
    a.columns = [c.strip().lstrip('#').strip().lower() for c in a.columns]
    def f(*names):
        for name in names:
            if name in a: return a[name].to_numpy(float)
        raise ValueError(f'Missing one of {names}')
    x,y=f('x_m','x'),f('y_m','y')
    wleft,wright=f('w_tr_left_m','w_tr_left','w_left'),f('w_tr_right_m','w_tr_right','w_right')
    if np.hypot(x[-1]-x[0],y[-1]-y[0]) < 1e-7:
        x,y,wleft,wright=x[:-1],y[:-1],wleft[:-1],wright[:-1]
    ds_sections=np.hypot(np.roll(x,-1)-x,np.roll(y,-1)-y)
    if ds_sections.min()<1e-5 or wleft.min()<=0 or wright.min()<=0:
        raise ValueError('Invalid consecutive XY points or width')
    if np.max(np.abs(ds_sections-np.median(ds_sections))) / np.median(ds_sections) > .15:
        raise ValueError('Nonuniform point spacing: resample centerline before using uniform finite differences.')
    # heading from periodic neighboring points, no discontinuity at start/finish
    theta=np.arctan2(np.roll(y,-1)-np.roll(y,1),np.roll(x,-1)-np.roll(x,1))
    nx,ny=-np.sin(theta),np.cos(theta)
    return x,y,nx,ny,wleft,wright,float(ds_sections.mean())


def build_derivatives(n,ds):
    first=sparse.lil_matrix((n,n));second=sparse.lil_matrix((n,n))
    for i in range(n):
        first[i,(i+1)%n]=1/(2*ds)
        first[i,(i-1)%n]=-1/(2*ds)
        second[i,(i-1)%n]=1/ds**2
        second[i,i]=-2/ds**2
        second[i,(i+1)%n]=1/ds**2
    return first.tocsc(),second.tocsc()


class CurvatureSQP:
    def __init__(self,track, ey_margin=0.0, exploration_tube=0.25,
                 smooth_weight=0.2,length_weight=0.01, damping=1e-3):
        x,y,nx,ny,wl,wr,ds=track
        self.x,self.y,self.nx,self.ny,self.ds=x,y,nx,ny,ds
        self.lo=-wr +ey_margin
        self.hi=wl -ey_margin
        if np.any(self.lo>0) or np.any(self.hi<0):
            raise ValueError('Margin exceeds track width.')
        self.e=np.zeros(len(x))
        self.tube=exploration_tube
        self.wk=max(smooth_weight,0.)
        self.wl=max(length_weight,0.)
        self.damping=damping
        self.D1,self.D2=build_derivatives(len(x),ds)
        N1=sparse.diags(nx,format='csc'); N2=sparse.diags(ny,format='csc')
        self.Vx=self.D1@N1;self.Vy=self.D1@N2
        self.Ax=self.D2@N1;self.Ay=self.D2@N2
        self.I=sparse.eye(len(x),format='csc')

    def xy(self,e):
        return self.x+self.nx*e,self.y+self.ny*e

    def terms(self,e, need_jac=True):
        x,y=self.xy(e)
        vx,vy=self.D1@x,self.D1@y
        ax,ay=self.D2@x,self.D2@y
        sp=np.sqrt(vx*vx+vy*vy+1e-10)
        det=vx*ay-vy*ax
        k=det/sp**3
        r=np.sqrt(self.ds)*det/sp**2.5
        u=self.D1@k
        rs=np.sqrt(self.wk*self.ds)*u/np.sqrt(sp)
        length=self.ds*np.sum(sp)
        cost=.5*float(r@r)+.5*float(rs@rs)+self.wl*length
        if not need_jac:
            return cost,k,length
        # Linearization of the TRUE geometric curvature, rather than XY''(s).
        jdot=(sparse.diags(vx)@self.Vx+sparse.diags(vy)@self.Vy).tocsc()
        jdet=(sparse.diags(ay)@self.Vx+sparse.diags(vx)@self.Ay-
              sparse.diags(ax)@self.Vy-sparse.diags(vy)@self.Ax).tocsc()
        jk=(sparse.diags(sp**-3)@jdet-
            sparse.diags(3*det/sp**5)@jdot).tocsc()
        jr=(np.sqrt(self.ds)*(sparse.diags(sp**-2.5)@jdet-
             sparse.diags(2.5*det/sp**4.5)@jdot)).tocsc()
        if self.wk:
            js=(np.sqrt(self.wk*self.ds)*(sparse.diags(sp**-0.5)@self.D1@jk-
                sparse.diags(0.5*u/sp**2.5)@jdot)).tocsc()
        else:
            js=sparse.csc_matrix((len(e),len(e)))
        gl=self.ds*(self.Vx.T@(vx/sp)+self.Vy.T@(vy/sp))
        P=(jr.T@jr+js.T@js+self.damping*self.I).tocsc()
        q=np.asarray(jr.T@r+js.T@rs+self.wl*gl).ravel()
        return cost,k,length,P,q

    def _qp(self,P,q,lo,hi):
        if osqp is not None:
            solver=osqp.OSQP()
            solver.setup(P=sparse.triu(P,format='csc'),q=q,A=self.I,
                         l=lo,u=hi,eps_abs=1e-6,eps_rel=1e-6,
                         max_iter=30000,verbose=False,polishing=True)
            sol=solver.solve()
            if sol.info.status_val not in (1,2):
                raise RuntimeError(f'OSQP did not solve SQP subproblem: {sol.info.status}')
            return sol.x,sol.info.status
        # QP solved with L-BFGS-B as offline fallback: convex quadratic
        sc=max(float(abs(P.diagonal()).max()),1.)
        def f(d):
            g=P@d+q
            return (.5*d@(P@d)+q@d)/sc,g/sc
        sol=minimize(f,np.zeros_like(q),method='L-BFGS-B',jac=True,
                     bounds=list(zip(lo,hi)),options={'maxiter':2500,'gtol':1e-9,'ftol':1e-13})
        return sol.x,f'SciPy QP ({sol.message})'

    def step(self,i):
        base,k,L,P,q=self.terms(self.e)
        delta,status=self._qp(P,q,np.maximum(self.lo-self.e,-self.tube),
                              np.minimum(self.hi-self.e,self.tube))
        # Damped SQP globalization: reject steps that increase TRUE objective.
        alpha=1.
        while alpha>=1/1024:
            candidate=np.minimum(self.hi,np.maximum(self.lo,self.e+alpha*delta))
            candcost,_,_=self.terms(candidate,False)
            if candcost < base-1e-9:break
            alpha*=.5
        if alpha<1/1024:
            candidate=self.e.copy();candcost=base
        move=np.max(np.abs(candidate-self.e))
        self.e=candidate
        x,y=self.xy(candidate)
        dsxy=np.hypot(np.roll(x,-1)-x,np.roll(y,-1)-y)
        near_wall=np.mean((candidate-self.lo<.02)|(self.hi-candidate<.02))
        print(f'iter {i:02d} cost={candcost:.4f}, max_step={move:.4f}m, '
              f'rms_offset={np.sqrt(np.mean(candidate**2)):.3f}m, '
              f'|ey|>0.5m={np.mean(np.abs(candidate)>.5)*100:.1f}%, '
              f'near_wall={near_wall*100:.1f}%, min_point_spacing={dsxy.min():.3f}m, '
              f'alpha={alpha:.3g}')
        return move


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('csv',nargs='?',default='/home/giorgos/sim_ws/src/f1tenth_gym_ros/maps/SaoPaulo_centerline.csv')
    ap.add_argument('--iterations',type=int,default=5)
    ap.add_argument('--margin',type=float,default=0.)
    ap.add_argument('--exploration',type=float,default=.25)
    ap.add_argument('--smooth',type=float,default=.2)
    ap.add_argument('--length-weight',type=float,default=.01)
    ap.add_argument('--damping',type=float,default=1e-3)
    ap.add_argument('--save',default='optimized_racing_line.csv')
    args=ap.parse_args()
    track=load_track(args.csv)
    solver=CurvatureSQP(track,args.margin,args.exploration,args.smooth,args.length_weight,args.damping)
    print('Running curvature-based SQP', 'OSQP' if osqp else 'SciPy QP fallback', 'npts=',len(track[0]))
    history=[]
    for i in range(1,args.iterations+1):
        change=solver.step(i)
        if i==1 or i%5==0:history.append((i,solver.xy(solver.e)))
        if change<1e-4:
            print('Stopping: small accepted step.');break
    xx,yy=solver.xy(solver.e)
    # Supply both centerline stations and actual path stations. The final XY
    # points are NOT uniformly spaced along the racing line.
    segments=np.hypot(np.roll(xx,-1)-xx,np.roll(yy,-1)-yy)
    s_path=np.r_[0.,np.cumsum(segments[:-1])]
    dx=solver.D1@xx;dy=solver.D1@yy
    ddx=solver.D2@xx;ddy=solver.D2@yy
    path_theta=np.arctan2(dy,dx)
    path_kappa=(dx*ddy-dy*ddx)/np.maximum((dx*dx+dy*dy)**1.5,1e-12)
    out=np.column_stack([np.arange(len(xx))*track[-1],s_path,xx,yy,solver.e,path_theta,path_kappa])
    np.savetxt(args.save,out,delimiter=',',header='s_centerline_m,s_path_m,x_m,y_m,e_y_m,theta_rad,kappa_1pm',comments='')
    print('Saved:',args.save)
    fig,ax=plt.subplots(figsize=(10,10))
    x,y,nx,ny,wl,wr,ds=track
    ax.plot(x+nx*wl,y+ny*wl,'k--',lw=1,label='Track limits')
    ax.plot(x-nx*wr,y-ny*wr,'k--',lw=1)
    ax.plot(x,y,':',color='.6',label='Centerline')
    for i,(xxi,yyi) in history:
        ax.plot(xxi,yyi,lw=.8,alpha=.25)
    ax.plot(xx,yy,'r',lw=2.2,label='Final line')
    ax.set_aspect('equal');ax.grid(alpha=.3);ax.legend();ax.set_title('Actual curvature SQP racing line')
    plt.show()


if __name__=='__main__':main()
