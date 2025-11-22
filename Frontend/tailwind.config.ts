import type { Config } from "tailwindcss";

// all in fixtures is set to tailwind v3 as interims solutions

const config: Config = {
    darkMode: ["class"],
    content: [
    "./pages/**/*.{js,ts,jsx,tsx,mdx}",
    "./components/**/*.{js,ts,jsx,tsx,mdx}",
    "./app/**/*.{js,ts,jsx,tsx,mdx}",
    "*.{js,ts,jsx,tsx,mdx}"
  ],
  theme: {
  	screens: {
  		// Mobile-first approach with comprehensive device support
  		'xs': '375px',      // iPhone SE, small Android phones
  		'sm': '640px',      // Large phones, small tablets
  		'md': '768px',      // Tablets (iPad mini, small tablets)
  		'lg': '1024px',     // Large tablets (iPad, iPad Air)
  		'xl': '1280px',     // Small laptops
  		'2xl': '1536px',    // Large laptops, desktops
  		'3xl': '1920px',    // Large desktops, 4K displays
  		'4xl': '2560px',    // Ultra-wide displays
  		
  		// Device-specific breakpoints
  		'phone': '375px',   // iPhone SE and similar
  		'phone-lg': '414px', // iPhone 12/13/14 Pro Max
  		'tablet': '768px',  // iPad, Android tablets
  		'tablet-lg': '1024px', // iPad Pro, large tablets
  		'laptop': '1280px', // Laptops
  		'desktop': '1536px', // Desktop monitors
  		'desktop-lg': '1920px', // Large desktop monitors
  		
  		// Orientation-specific
  		'portrait': {'raw': '(orientation: portrait)'},
  		'landscape': {'raw': '(orientation: landscape)'},
  		
  		// Touch device detection
  		'touch': {'raw': '(pointer: coarse)'},
  		'no-touch': {'raw': '(pointer: fine)'},
  		
  		// High DPI displays
  		'retina': {'raw': '(-webkit-min-device-pixel-ratio: 2), (min-resolution: 192dpi)'},
  	},
  	extend: {
  		colors: {
  			background: 'hsl(var(--background))',
  			foreground: 'hsl(var(--foreground))',
  			'custom-gray': {
  				dark: '#202529',
  				medium: '#32383D'
  			},
  			card: {
  				DEFAULT: 'hsl(var(--card))',
  				foreground: 'hsl(var(--card-foreground))'
  			},
  			popover: {
  				DEFAULT: 'hsl(var(--popover))',
  				foreground: 'hsl(var(--popover-foreground))'
  			},
  			primary: {
  				DEFAULT: 'hsl(var(--primary))',
  				foreground: 'hsl(var(--primary-foreground))',
  				hover: 'hsl(var(--primary-hover))',
  				light: 'hsl(var(--primary-light))'
  			},
  			secondary: {
  				DEFAULT: 'hsl(var(--secondary))',
  				foreground: 'hsl(var(--secondary-foreground))',
  				hover: 'hsl(var(--secondary-hover))',
  				light: 'hsl(var(--secondary-light))'
  			},
  			accent: {
  				DEFAULT: 'hsl(var(--accent))',
  				foreground: 'hsl(var(--accent-foreground))',
  				hover: 'hsl(var(--accent-hover))',
  				light: 'hsl(var(--accent-light))'
  			},
  			success: {
  				DEFAULT: 'hsl(var(--success))',
  				foreground: 'hsl(var(--success-foreground))',
  				hover: 'hsl(var(--success-hover))',
  				light: 'hsl(var(--success-light))'
  			},
  			warning: {
  				DEFAULT: 'hsl(var(--warning))',
  				foreground: 'hsl(var(--warning-foreground))',
  				hover: 'hsl(var(--warning-hover))',
  				light: 'hsl(var(--warning-light))'
  			},
  			destructive: {
  				DEFAULT: 'hsl(var(--destructive))',
  				foreground: 'hsl(var(--destructive-foreground))',
  				hover: 'hsl(var(--destructive-hover))',
  				light: 'hsl(var(--destructive-light))'
  			},
  			muted: {
  				DEFAULT: 'hsl(var(--muted))',
  				foreground: 'hsl(var(--muted-foreground))'
  			},
  			border: 'hsl(var(--border))',
  			input: 'hsl(var(--input))',
  			ring: 'hsl(var(--ring))',
  			chart: {
  				'1': 'hsl(var(--chart-1))',
  				'2': 'hsl(var(--chart-2))',
  				'3': 'hsl(var(--chart-3))',
  				'4': 'hsl(var(--chart-4))',
  				'5': 'hsl(var(--chart-5))'
  			},
  			status: {
  				active: 'hsl(var(--status-active))',
  				maintenance: 'hsl(var(--status-maintenance))',
  				inactive: 'hsl(var(--status-inactive))',
  				pending: 'hsl(var(--status-pending))'
  			},
  			sidebar: {
  				DEFAULT: 'hsl(var(--sidebar-background))',
  				foreground: 'hsl(var(--sidebar-foreground))',
  				primary: 'hsl(var(--sidebar-primary))',
  				'primary-foreground': 'hsl(var(--sidebar-primary-foreground))',
  				accent: 'hsl(var(--sidebar-accent))',
  				'accent-foreground': 'hsl(var(--sidebar-accent-foreground))',
  				border: 'hsl(var(--sidebar-border))',
  				ring: 'hsl(var(--sidebar-ring))'
  			}
  		},
  		borderRadius: {
  			lg: 'var(--radius)',
  			md: 'calc(var(--radius) - 2px)',
  			sm: 'calc(var(--radius) - 4px)'
  		},
  		boxShadow: {
  			'sm': 'var(--shadow-sm)',
  			'DEFAULT': 'var(--shadow)',
  			'md': 'var(--shadow-md)',
  			'lg': 'var(--shadow-lg)',
  			'xl': 'var(--shadow-xl)',
  		},
  		spacing: {
  			'0.75': '0.1875rem',
  			// Responsive spacing utilities
  			'18': '4.5rem',    // 72px
  			'88': '22rem',     // 352px - sidebar width
  			'112': '28rem',    // 448px
  			'128': '32rem',    // 512px
  		},
  		fontSize: {
  			// Responsive font sizes
  			'xs': ['0.75rem', { lineHeight: '1rem' }],
  			'sm': ['0.875rem', { lineHeight: '1.25rem' }],
  			'base': ['1rem', { lineHeight: '1.5rem' }],
  			'lg': ['1.125rem', { lineHeight: '1.75rem' }],
  			'xl': ['1.25rem', { lineHeight: '1.75rem' }],
  			'2xl': ['1.5rem', { lineHeight: '2rem' }],
  			'3xl': ['1.875rem', { lineHeight: '2.25rem' }],
  			'4xl': ['2.25rem', { lineHeight: '2.5rem' }],
  			'5xl': ['3rem', { lineHeight: '1' }],
  			'6xl': ['3.75rem', { lineHeight: '1' }],
  			'7xl': ['4.5rem', { lineHeight: '1' }],
  			'8xl': ['6rem', { lineHeight: '1' }],
  			'9xl': ['8rem', { lineHeight: '1' }],
  		},
  		keyframes: {
			'accordion-down': {
				from: {
					height: '0'
				},
				to: {
					height: 'var(--radix-accordion-content-height)'
				}
			},
			'accordion-up': {
				from: {
					height: 'var(--radix-accordion-content-height)'
				},
				to: {
					height: '0'
				}
			},
			'float': {
				'0%, 100%': {
					transform: 'translateY(0px)'
				},
				'50%': {
					transform: 'translateY(-10px)'
				}
			},
			'glow': {
				'0%, 100%': {
					opacity: '1'
				},
				'50%': {
					opacity: '0.8'
				}
			}
		},
		animation: {
			'accordion-down': 'accordion-down 0.2s ease-out',
			'accordion-up': 'accordion-up 0.2s ease-out',
			'float': 'float 6s ease-in-out infinite',
			'glow': 'glow 2s ease-in-out infinite'
		},
		extend: {
			borderWidth: {
				'0.75': '0.1875rem',
			}
		}
  	}
  },
  plugins: [require("tailwindcss-animate")],
};
export default config;
