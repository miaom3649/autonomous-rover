# Headless stub — satisfies picamera2's import of DrmPreview without real KMS/DRM hardware.
# DrmPreview is never instantiated in capture-only (no-display) mode.

class PixelFormat:
    RGB888 = None
    BGR888 = None
    XRGB8888 = None
    XBGR8888 = None
    ABGR8888 = None
    YUV420 = None
    YVU420 = None

class Card:
    pass

class ResourceManager:
    pass

class DumbFramebuffer:
    pass

class DmabufFramebuffer:
    pass

class AtomicReq:
    pass

class PlaneType:
    Primary = None
