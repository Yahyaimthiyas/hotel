import * as React from "react"

// Enhanced breakpoints for comprehensive device detection
const BREAKPOINTS = {
  xs: 375,      // iPhone SE, small Android phones
  sm: 640,      // Large phones, small tablets
  md: 768,      // Tablets (iPad mini, small tablets)
  lg: 1024,     // Large tablets (iPad, iPad Air)
  xl: 1280,     // Small laptops
  '2xl': 1536,  // Large laptops, desktops
  '3xl': 1920,  // Large desktops, 4K displays
  '4xl': 2560,  // Ultra-wide displays
}

const MOBILE_BREAKPOINT = 768
const TABLET_BREAKPOINT = 1024
const DESKTOP_BREAKPOINT = 1280

export function useIsMobile() {
  const [isMobile, setIsMobile] = React.useState<boolean | undefined>(undefined)

  React.useEffect(() => {
    const mql = window.matchMedia(`(max-width: ${MOBILE_BREAKPOINT - 1}px)`)
    const onChange = () => {
      setIsMobile(window.innerWidth < MOBILE_BREAKPOINT)
    }
    mql.addEventListener("change", onChange)
    setIsMobile(window.innerWidth < MOBILE_BREAKPOINT)
    return () => mql.removeEventListener("change", onChange)
  }, [])

  return !!isMobile
}

export function useDeviceType() {
  const [deviceType, setDeviceType] = React.useState<'mobile' | 'tablet' | 'desktop' | undefined>(undefined)
  const [screenSize, setScreenSize] = React.useState<{
    width: number
    height: number
    isPortrait: boolean
    isLandscape: boolean
  }>({ width: 0, height: 0, isPortrait: false, isLandscape: false })

  React.useEffect(() => {
    const updateDeviceInfo = () => {
      const width = window.innerWidth
      const height = window.innerHeight
      const isPortrait = height > width
      const isLandscape = width > height

      setScreenSize({ width, height, isPortrait, isLandscape })

      if (width < MOBILE_BREAKPOINT) {
        setDeviceType('mobile')
      } else if (width < TABLET_BREAKPOINT) {
        setDeviceType('tablet')
      } else {
        setDeviceType('desktop')
      }
    }

    updateDeviceInfo()
    window.addEventListener('resize', updateDeviceInfo)
    window.addEventListener('orientationchange', updateDeviceInfo)

    return () => {
      window.removeEventListener('resize', updateDeviceInfo)
      window.removeEventListener('orientationchange', updateDeviceInfo)
    }
  }, [])

  return { deviceType, screenSize }
}

export function useBreakpoint() {
  const [currentBreakpoint, setCurrentBreakpoint] = React.useState<string>('xs')
  const [isAboveBreakpoint, setIsAboveBreakpoint] = React.useState<Record<string, boolean>>({})

  React.useEffect(() => {
    const updateBreakpoints = () => {
      const width = window.innerWidth
      let newBreakpoint = 'xs'

      // Determine current breakpoint
      if (width >= BREAKPOINTS['4xl']) newBreakpoint = '4xl'
      else if (width >= BREAKPOINTS['3xl']) newBreakpoint = '3xl'
      else if (width >= BREAKPOINTS['2xl']) newBreakpoint = '2xl'
      else if (width >= BREAKPOINTS.xl) newBreakpoint = 'xl'
      else if (width >= BREAKPOINTS.lg) newBreakpoint = 'lg'
      else if (width >= BREAKPOINTS.md) newBreakpoint = 'md'
      else if (width >= BREAKPOINTS.sm) newBreakpoint = 'sm'
      else newBreakpoint = 'xs'

      setCurrentBreakpoint(newBreakpoint)

      // Calculate which breakpoints we're above
      const aboveBreakpoints: Record<string, boolean> = {}
      Object.entries(BREAKPOINTS).forEach(([breakpoint, minWidth]) => {
        aboveBreakpoints[breakpoint] = width >= minWidth
      })
      setIsAboveBreakpoint(aboveBreakpoints)
    }

    updateBreakpoints()
    window.addEventListener('resize', updateBreakpoints)
    window.addEventListener('orientationchange', updateBreakpoints)

    return () => {
      window.removeEventListener('resize', updateBreakpoints)
      window.removeEventListener('orientationchange', updateBreakpoints)
    }
  }, [])

  return { currentBreakpoint, isAboveBreakpoint }
}

export function useTouchDevice() {
  const [isTouchDevice, setIsTouchDevice] = React.useState<boolean>(false)

  React.useEffect(() => {
    const checkTouchDevice = () => {
      const isTouch = 'ontouchstart' in window || navigator.maxTouchPoints > 0
      setIsTouchDevice(isTouch)
    }

    checkTouchDevice()
    window.addEventListener('touchstart', checkTouchDevice, { once: true })

    return () => {
      window.removeEventListener('touchstart', checkTouchDevice)
    }
  }, [])

  return isTouchDevice
}

export function useOrientation() {
  const [orientation, setOrientation] = React.useState<'portrait' | 'landscape'>('portrait')

  React.useEffect(() => {
    const updateOrientation = () => {
      const isPortrait = window.innerHeight > window.innerWidth
      setOrientation(isPortrait ? 'portrait' : 'landscape')
    }

    updateOrientation()
    window.addEventListener('resize', updateOrientation)
    window.addEventListener('orientationchange', updateOrientation)

    return () => {
      window.removeEventListener('resize', updateOrientation)
      window.removeEventListener('orientationchange', updateOrientation)
    }
  }, [])

  return orientation
}

// Comprehensive device detection hook
export function useDeviceDetection() {
  const { deviceType, screenSize } = useDeviceType()
  const { currentBreakpoint, isAboveBreakpoint } = useBreakpoint()
  const isTouchDevice = useTouchDevice()
  const orientation = useOrientation()
  const isMobile = useIsMobile()

  return {
    // Device type
    deviceType,
    isMobile,
    isTablet: deviceType === 'tablet',
    isDesktop: deviceType === 'desktop',
    
    // Screen information
    screenSize,
    orientation,
    isPortrait: orientation === 'portrait',
    isLandscape: orientation === 'landscape',
    
    // Breakpoint information
    currentBreakpoint,
    isAboveBreakpoint,
    
    // Device capabilities
    isTouchDevice,
    
    // Convenience helpers
    isSmallScreen: screenSize.width < 640,
    isMediumScreen: screenSize.width >= 640 && screenSize.width < 1024,
    isLargeScreen: screenSize.width >= 1024,
    
    // Responsive helpers
    shouldShowMobileLayout: deviceType === 'mobile' || (deviceType === 'tablet' && orientation === 'portrait'),
    shouldShowTabletLayout: deviceType === 'tablet' && orientation === 'landscape',
    shouldShowDesktopLayout: deviceType === 'desktop',
  }
}
