// Shared hotel data that can be updated across components
export const mockHotelsData = {
  "1": {
    id: "1",
    name: "Coastal Grand Hotel - Ooty",
    location: "Ooty, Tamil Nadu",
    managerName: "Rajesh Kumar",
    totalRooms: 25,
    activeRooms: 21,
    totalCards: 23,
    status: "active",
    occupancy: 84,
    lastActivity: "2 minutes ago",
    image: "/placeholder.jpg",
    description: "Scenic hill station hotel with modern amenities and exceptional service.",
    phone: "+91 90476 28844",
    email: "rajesh.kumar@coastalgrand.com",
    address: "456 Hill Road, Ooty, Tamil Nadu",
    rating: 4.7,
    manager: {
      name: "Rajesh Kumar",
      phone: "+91 90476 28844",
      email: "rajesh.kumar@coastalgrand.com",
      status: "online"
    }
  },
  "2": {
    id: "2",
    name: "Coastal Grand Hotel - Salem",
    location: "Salem, Tamil Nadu",
    managerName: "Priya Devi",
    totalRooms: 30,
    activeRooms: 25,
    totalCards: 28,
    status: "active",
    occupancy: 83,
    lastActivity: "5 minutes ago",
    image: "/placeholder.jpg",
    description: "Premium hotel in the heart of Salem with modern amenities and exceptional service.",
    phone: "+91 90476 28844",
    email: "priya.devi@coastalgrand.com",
    address: "123 Main Street, Salem, Tamil Nadu",
    rating: 4.8,
    manager: {
      name: "Priya Devi",
      phone: "+91 90476 28844",
      email: "priya.devi@coastalgrand.com",
      status: "online"
    }
  },
  "3": {
    id: "3",
    name: "Coastal Grand Hotel - Yercaud",
    location: "Yercaud, Tamil Nadu",
    managerName: "Arun Balaji",
    totalRooms: 20,
    activeRooms: 18,
    totalCards: 19,
    status: "active",
    occupancy: 90,
    lastActivity: "10 minutes ago",
    image: "/placeholder.jpg",
    description: "Scenic hill station hotel with modern amenities and exceptional service.",
    phone: "+91 90476 28844",
    email: "arun.balaji@coastalgrand.com",
    address: "789 Mountain View, Yercaud, Tamil Nadu",
    rating: 4.6,
    manager: {
      name: "Arun Balaji",
      phone: "+91 90476 28844",
      email: "arun.balaji@coastalgrand.com",
      status: "online"
    }
  },
  "4": {
    id: "4",
    name: "Coastal Grand Hotel - Puducherry",
    location: "Puducherry, Union Territory",
    managerName: "Lakshmi Priya",
    totalRooms: 28,
    activeRooms: 20,
    totalCards: 25,
    status: "maintenance",
    occupancy: 71,
    lastActivity: "1 hour ago",
    image: "/placeholder.jpg",
    description: "Heritage hotel with modern amenities and exceptional service.",
    phone: "+91 90476 28844",
    email: "lakshmi.priya@coastalgrand.com",
    address: "321 Beach Road, Puducherry, Union Territory",
    rating: 4.5,
    manager: {
      name: "Lakshmi Priya",
      phone: "+91 90476 28844",
      email: "lakshmi.priya@coastalgrand.com",
      status: "online"
    }
  },
  "5": {
    id: "5",
    name: "Coastal Grand Hotel - Namakkal",
    location: "Namakkal, Tamil Nadu",
    managerName: "Senthil Kumar",
    totalRooms: 22,
    activeRooms: 18,
    totalCards: 20,
    status: "active",
    occupancy: 82,
    lastActivity: "15 minutes ago",
    image: "/placeholder.jpg",
    description: "Premium hotel with modern amenities and exceptional service.",
    phone: "+91 90476 28844",
    email: "senthil.kumar@coastalgrand.com",
    address: "654 City Center, Namakkal, Tamil Nadu",
    rating: 4.4,
    manager: {
      name: "Senthil Kumar",
      phone: "+91 90476 28844",
      email: "senthil.kumar@coastalgrand.com",
      status: "online"
    }
  },
  "6": {
    id: "6",
    name: "Coastal Grand Hotel - Chennai",
    location: "Chennai, Tamil Nadu",
    managerName: "Vijay Anand",
    totalRooms: 30,
    activeRooms: 27,
    totalCards: 28,
    status: "active",
    occupancy: 90,
    lastActivity: "30 minutes ago",
    image: "/placeholder.jpg",
    description: "Metropolitan hotel with modern amenities and exceptional service.",
    phone: "+91 90476 28844",
    email: "vijay.anand@coastalgrand.com",
    address: "987 Marina Beach Road, Chennai, Tamil Nadu",
    rating: 4.9,
    manager: {
      name: "Vijay Anand",
      phone: "+91 90476 28844",
      email: "vijay.anand@coastalgrand.com",
      status: "online"
    }
  },
  "7": {
    id: "7",
    name: "Coastal Grand Hotel - Bangalore",
    location: "Bangalore, Karnataka",
    managerName: "Deepa Sharma",
    totalRooms: 30,
    activeRooms: 26,
    totalCards: 28,
    status: "active",
    occupancy: 87,
    lastActivity: "45 minutes ago",
    image: "/placeholder.jpg",
    description: "Metropolitan hotel with modern amenities and exceptional service.",
    phone: "+91 90476 28844",
    email: "deepa.sharma@coastalgrand.com",
    address: "147 MG Road, Bangalore, Karnataka",
    rating: 4.7,
    manager: {
      name: "Deepa Sharma",
      phone: "+91 90476 28844",
      email: "deepa.sharma@coastalgrand.com",
      status: "online"
    }
  },
  "8": {
    id: "8",
    name: "Coastal Grand Hotel - Kotagiri",
    location: "Kotagiri, Tamil Nadu",
    managerName: "Mohan Raj",
    totalRooms: 18,
    activeRooms: 16,
    totalCards: 17,
    status: "active",
    occupancy: 89,
    lastActivity: "1 hour ago",
    image: "/placeholder.jpg",
    description: "Scenic hill station hotel with modern amenities and exceptional service.",
    phone: "+91 90476 28844",
    email: "mohan.raj@coastalgrand.com",
    address: "258 Tea Estate Road, Kotagiri, Tamil Nadu",
    rating: 4.6,
    manager: {
      name: "Mohan Raj",
      phone: "+91 90476 28844",
      email: "mohan.raj@coastalgrand.com",
      status: "online"
    }
  },
}

// Function to update hotel data
export const updateHotelData = (hotelId: string, updatedHotel: any) => {
  mockHotelsData[hotelId as keyof typeof mockHotelsData] = updatedHotel
  
  // Dispatch a custom event to notify other components
  const event = new CustomEvent('hotelDataUpdated', { 
    detail: { hotelId, updatedHotel } 
  })
  window.dispatchEvent(event)
}

// Function to get all hotels as an array for the overview
export const getAllHotels = () => {
  return Object.values(mockHotelsData)
}

// Function to get a specific hotel
export const getHotelById = (hotelId: string) => {
  return mockHotelsData[hotelId as keyof typeof mockHotelsData]
} 