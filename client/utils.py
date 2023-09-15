def stateToDict(state) -> dict:
    dict = {
        'x': state.contents.lX,
        'y': state.contents.lY,
        'z': state.contents.lZ,
        'Rx': state.contents.lRx,
        'Ry': state.contents.lRy,
        'Rz': state.contents.lRz,
        'slider': state.contents.rglSlider[:],
        'pov': state.contents.rgdwPOV[:],
        'buttons': state.contents.rgbButtons[:]
    }
    return dict

