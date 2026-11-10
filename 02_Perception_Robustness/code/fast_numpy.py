"""Exact small-array/scalar shortcuts, with ordinary NumPy as fallback.
No change to equations or integration rate. Used equally in every version.
Original functions remain available for parity validation.
"""
import numpy as np
_original_clip=np.clip
_original_norm=np.linalg.norm

def clip(a,a_min=None,a_max=None,out=None,**kw):
    if (type(a) in (float,np.float64) and a!=0 and a==a and out is None and not kw
        and type(a_min) in (int,float,np.float64) and type(a_max) in (int,float,np.float64)
        and a_min==a_min and a_max==a_max):
        return np.float64(min(max(a,a_min),a_max))
    return _original_clip(a,a_min,a_max,out=out,**kw)

def norm(a,ord=None,axis=None,keepdims=False):
    if (ord is None and axis is None and not keepdims and isinstance(a,np.ndarray)
        and a.shape==(2,) and a.dtype==np.float64 and a.flags.c_contiguous):
        return np.sqrt(a.dot(a))
    return _original_norm(a,ord=ord,axis=axis,keepdims=keepdims)

def install():
    np.clip=clip
    np.linalg.norm=norm
