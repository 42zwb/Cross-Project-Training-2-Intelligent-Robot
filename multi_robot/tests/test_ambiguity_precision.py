import numpy as np
import pytest
import fleet_mapping.registration as reg

@pytest.mark.parametrize('runner_score,runner_rms',[(.79,.04),(.81,.07),(.79,.07)])
def test_precision_gate_does_not_hide_rival(monkeypatch,runner_score,runner_rms):
    data=np.zeros((40,40),np.int16);data[:3,:]=100;data[:,:3]=100
    a=reg.Snapshot('robot1','a',1,1,data,.05,(0,0,0))
    b=reg.Snapshot('robot2','b',1,1,data,.05,(0,0,0))
    monkeypatch.setattr(reg,'distinctive_support',lambda *a:True)
    monkeypatch.setattr(reg,'intrinsic_ambiguity',lambda *a:False)
    monkeypatch.setattr(reg,'_features',lambda *a:([],None))
    monkeypatch.setattr(reg,'_geometry_hypotheses',lambda *a:[(0.,0.,0.),(3.,0.,0.)])
    monkeypatch.setattr(reg,'_refine_edges',lambda a,b,p:p)
    monkeypatch.setattr(reg,'assess',lambda a,b,p:(.85,.5,.04) if p[0]==0 else (runner_score,.5,runner_rms))
    assert reg.register(a,b) is None
