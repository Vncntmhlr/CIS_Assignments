"""Five publication-oriented figure groups built only from saved real attempts."""

from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
COLORS={'nominal':'#31688e','improved':'#d95f02'}
LABELS={'random':'Random full range','limits':'Joint limits','boundary':'Workspace boundary','singularity':'Wrist singularity'}


def selected(reports,family,model,category=None):
    return [r for report in reports for r in report['rows'] if r['family']==family and r['model']==model
            and (category is None or r['category']==category)]


def empirical_cdf(rows,column):
    """Finite CDF has attempted denominator; failed attempts leave missing mass."""
    values=np.sort([r['error'][column] for r in rows if r.get('error') is not None])
    return values,np.arange(1,len(values)+1)/len(rows) if rows else np.array([])


def generate_figures(reports,stage):
    plt.rcParams.update({'font.size':9,'axes.titlesize':10,'axes.labelsize':9,
                         'legend.fontsize':8,'pdf.fonttype':42,'savefig.dpi':220})
    categories=list(LABELS)
    title=f'{stage.upper()}: seeds '+', '.join(str(r['seed']) for r in reports)
    figures=[]
    fig,axes=plt.subplots(2,2,figsize=(10,7),layout='constrained')
    for j,family in enumerate(['physical_fk','physical_ik']):
        for k,(column,target,unit) in enumerate([(1,.2,'Position error (mm)'),(0,.001,'Orientation error (rad)')]):
            ax=axes[j,k]
            for model in COLORS:
                random=selected(reports,family,model,'random')
                cohorts=[('Random ordinary subset',[r for r in random if r['ordinary']],'-'),
                         ('Designed stress',[r for r in selected(reports,family,model) if r['category']!='random'],'--')]
                for cohort,rows,style in cohorts:
                    x,y=empirical_cdf(rows,column)
                    missing=sum(r.get('error') is None for r in rows)
                    ax.step(x,y,where='post',color=COLORS[model],ls=style,
                            label=f'{model}, {cohort.lower()} ({missing}/{len(rows)} unavailable)')
            ax.axvline(target,color='black',ls=':',label='Accuracy target')
            ax.set(xscale='log',xlabel=unit,ylabel='Fraction of attempted poses',ylim=(0,1.03),
                   title='Physical FK' if family=='physical_fk' else 'Executed physical IK')
            ax.grid(alpha=.2);ax.legend(loc='lower right',fontsize=6)
    fig.suptitle(title+' — attempted-target CDFs; missing mass is not discarded')
    figures.append(('cdfs',fig))

    fig,axes=plt.subplots(2,2,figsize=(10,7),layout='constrained')
    for j,family in enumerate(['physical_fk','physical_ik']):
        for k,(column,target,label) in enumerate([(1,.2,'Position error (mm)'),(0,.001,'Orientation error (rad)')]):
            ax=axes[j,k]
            for i,category in enumerate(categories):
                for m,model in enumerate(COLORS):
                    rows=selected(reports,family,model,category)
                    values=np.array([r['error'][column] for r in rows if r.get('error') is not None])
                    x=i+(-.15 if m==0 else .15)
                    if len(values):
                        box=ax.boxplot([values],positions=[x],widths=.24,showfliers=False,patch_artist=True,manage_ticks=False)
                        box['boxes'][0].set(facecolor=COLORS[model],alpha=.25)
                        # Show every finite attempt; deterministic offsets prevent overplotting.
                        ax.scatter(x+np.linspace(-.08,.08,len(values)),values,s=5,color=COLORS[model],alpha=.45,
                                   label=model if i==0 else None)
                    unavailable=sum(r.get('error') is None for r in rows)
                    ax.text(x,.98,f'{unavailable}/{len(rows)}',ha='center',va='top',fontsize=7,
                            transform=ax.get_xaxis_transform(),rotation=90)
            ax.axhline(target,color='black',ls=':')
            ax.set(yscale='log',ylabel=label,title='Physical FK' if j==0 else 'Executed physical IK',
                   xticks=range(4),xticklabels=['Random','Limits','Boundary','Singularity'])
            ax.tick_params(axis='x',rotation=12);ax.grid(axis='y',alpha=.2)
            if j==0 and k==0:ax.legend(loc='lower left')
    fig.suptitle(title+' — all finite attempts; labels show unavailable/attempted')
    figures.append(('categories',fig))

    # Workspace views are per-seed, not a misleading overlay of different robots.
    fig=plt.figure(figsize=(10,4.3*len(reports)),layout='constrained')
    for j,report in enumerate(reports):
        fk=[r for r in report['rows'] if r['family']=='physical_fk' and r['model']=='improved']
        xyz=np.asarray([r['workspace_position'] for r in fk])
        for i,(column,target,label) in enumerate([(1,.2,'Position error (mm)'),(0,.001,'Orientation error (rad)')]):
            ax=fig.add_subplot(len(reports),2,2*j+i+1,projection='3d')
            values=np.asarray([r['error'][column] for r in fk])
            maximum=max(target,max(r['error'][column] for p in reports for r in p['rows']
                        if r['family']=='physical_fk' and r['model']=='improved'))
            points=ax.scatter(*xyz.T,c=values,cmap='viridis',s=13,vmin=0,vmax=maximum,alpha=.8)
            ax.set(xlabel='Workspace x (mm)',ylabel='Workspace y (mm)',zlabel='Workspace z (mm)',
                   title=f'Seed {report["seed"]}: {label}')
            ax.set_box_aspect(np.ptp(xyz,axis=0))
            fig.colorbar(points,ax=ax,shrink=.65,label=label,pad=.08)
    fig.suptitle(f'{stage.upper()}: improved physical FK, all design poses, common color scales')
    figures.append(('workspace',fig))

    fig,axes=plt.subplots(2,3,figsize=(11,6.3),layout='constrained')
    for k,ax in enumerate(axes.flat):
        column=1 if k<3 else 0;target=.2 if k<3 else .001
        for report in reports:
            for category,marker in zip(categories,['.','s','^','x']):
                rows=[r for r in report['rows'] if r['family']=='physical_fk' and r['model']=='improved' and r['category']==category]
                ax.scatter([r['theta'][k] for r in rows],[r['error'][column] for r in rows],s=9,alpha=.5,
                           marker=marker,label=f'seed {report["seed"]}, {category}')
        ax.axhline(target,color='black',ls=':')
        ax.set(yscale='log',xlabel=f'Joint {k+1} ({"mm" if k<3 else "rad"})',
               ylabel='Position error (mm)' if k<3 else 'Orientation error (rad)')
        ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=6,ncols=2,loc='lower left')
    fig.suptitle(title+' — improved physical FK versus joints; individual poses')
    figures.append(('joints',fig))

    fig,axes=plt.subplots(1,2,figsize=(10,4.4),layout='constrained')
    sim_stages=['Historical calibration','New calibration','State replay','Tracker test','IK test']
    wall_stages=['Initialization (incl. new fit)','FK evaluation','Tracker test','Numerical IK','Physical IK']
    for report in reports:
        p,c=report['preparation'],report['costs'];name=f'Seed {report["seed"]}'
        values=[p['historical_calibration_s'] if report['seed']==1 else 0,p['acquisition_s'],p['replay_s'],
                c['tracker_prediction']['simulated_s'],c['physical_ik']['simulated_s']]
        left=0
        for i,v in enumerate(values):
            axes[0].barh(name,v/60,left=left/60,label=sim_stages[i] if report is reports[0] else None,
                         color=plt.get_cmap('tab10')(i),hatch='//' if i==0 else None)
            left+=v
        values=[p['initialization_wall_s'],c['physical_fk']['wall_s'],c['tracker_prediction']['wall_s'],
                c['numerical_consistency']['wall_s'],c['physical_ik']['wall_s']]
        left=0
        for i,v in enumerate(values):
            axes[1].barh(name,v,left=left,label=wall_stages[i] if report is reports[0] else None,color=plt.get_cmap('tab10')(i))
            left+=v
    axes[0].set(xlabel='Simulated time (minutes)',title='Historical cost and this run: distinguish hatched archive')
    axes[1].set(xlabel='Wall-clock time (seconds)',title='Measured pilot stages; no double-counted fitting')
    for ax in axes:ax.legend(fontsize=6,loc='upper center',bbox_to_anchor=(.5,-.2),ncols=2);ax.grid(axis='x',alpha=.2)
    fig.suptitle(title+' — simulated and wall-clock costs are different quantities')
    figures.append(('timing',fig))

    for name,fig in figures:
        prefix=ROOT/'results'/f'comprehensive_v1_{stage}_{name}'
        fig.savefig(prefix.with_suffix('.png'),bbox_inches='tight',pad_inches=.15)
        fig.savefig(prefix.with_suffix('.pdf'),bbox_inches='tight',pad_inches=.15)
        plt.close(fig)
