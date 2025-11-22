import { useState, useEffect } from 'react'

export function useScrollDirection() {
  const [scrollDirection, setScrollDirection] = useState<'up' | 'down' | null>(null)
  const [prevScrollY, setPrevScrollY] = useState(0)
  const [isAtTop, setIsAtTop] = useState(true)

  useEffect(() => {
    const handleScroll = () => {
      const currentScrollY = window.scrollY
      
      // Check if at top
      setIsAtTop(currentScrollY < 10)
      
      // Determine scroll direction
      if (currentScrollY > prevScrollY && currentScrollY > 100) {
        setScrollDirection('down')
      } else if (currentScrollY < prevScrollY) {
        setScrollDirection('up')
      }
      
      setPrevScrollY(currentScrollY)
    }

    // Throttle scroll events for better performance
    let ticking = false
    const throttledHandleScroll = () => {
      if (!ticking) {
        requestAnimationFrame(() => {
          handleScroll()
          ticking = false
        })
        ticking = true
      }
    }

    window.addEventListener('scroll', throttledHandleScroll, { passive: true })
    
    return () => {
      window.removeEventListener('scroll', throttledHandleScroll)
    }
  }, [prevScrollY])

  return { scrollDirection, isAtTop }
} 