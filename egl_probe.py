#!/usr/bin/env -S uv run python
# Enumerate EGL devices under the mesa ICD and report each device's DRM render node +
# GL_RENDERER, so we can pick the AMD iGPU (renderD129) index for render.c option B.
import os, ctypes
os.environ['__EGL_VENDOR_LIBRARY_FILENAMES'] = '/usr/share/glvnd/egl_vendor.d/50_mesa.json'

egl = ctypes.CDLL('libEGL.so.1')
gl  = ctypes.CDLL('libGL.so.1')
egl.eglGetProcAddress.restype = ctypes.c_void_p
egl.eglGetProcAddress.argtypes = [ctypes.c_char_p]
def proc(name, restype, *args):
    addr = egl.eglGetProcAddress(name.encode())
    return ctypes.CFUNCTYPE(restype, *args)(addr)

EGL_DRM_DEVICE_FILE_EXT      = 0x3233
EGL_DRM_RENDER_NODE_FILE_EXT = 0x3377
EGL_PLATFORM_DEVICE_EXT      = 0x313F
EGL_VENDOR = 0x3055; EGL_NO_DISPLAY = ctypes.c_void_p(0)

queryDevices = proc('eglQueryDevicesEXT', ctypes.c_uint,
                    ctypes.c_int, ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_int))
queryDevStr  = proc('eglQueryDeviceStringEXT', ctypes.c_char_p, ctypes.c_void_p, ctypes.c_int)
getPlatDisp  = proc('eglGetPlatformDisplayEXT', ctypes.c_void_p,
                    ctypes.c_uint, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int))
egl.eglInitialize.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
egl.eglQueryString.restype = ctypes.c_char_p; egl.eglQueryString.argtypes = [ctypes.c_void_p, ctypes.c_int]

devs = (ctypes.c_void_p * 8)(); n = ctypes.c_int(0)
if not queryDevices(8, devs, ctypes.byref(n)):
    print('eglQueryDevicesEXT failed'); raise SystemExit
print(f'mesa sees {n.value} EGL device(s):')
for i in range(n.value):
    d = devs[i]
    drm  = queryDevStr(d, EGL_DRM_DEVICE_FILE_EXT)
    node = queryDevStr(d, EGL_DRM_RENDER_NODE_FILE_EXT)
    dpy = getPlatDisp(EGL_PLATFORM_DEVICE_EXT, d, None)
    vendor = b'?'
    if dpy and dpy != 0:
        maj, mn = ctypes.c_int(0), ctypes.c_int(0)
        if egl.eglInitialize(dpy, ctypes.byref(maj), ctypes.byref(mn)):
            vendor = egl.eglQueryString(dpy, EGL_VENDOR) or b'?'
    print(f'  idx={i}: render_node={ (node or b"-").decode():18s} drm={ (drm or b"-").decode():14s} vendor={vendor.decode()}')
print('\n-> pick the idx whose render_node is /dev/dri/renderD129 (amdgpu iGPU) for option B.')
