import math

class MiroDepthCalculator:
    def __init__(self):
        # Constants from Ling 2019 ROBIO Paper & MiRo Hardware Specs
        self.BASELINE = 0.104       # Distance between eyes in meters
        self.FOCAL_LENGTH = 184.75  # Focal length in pixels
        
        # Camera Resolution (640x360 as per paper)
        self.IMG_WIDTH = 640
        self.IMG_HEIGHT = 360
        
        # Principal
        self.cx = self.IMG_WIDTH / 2
        self.cy = self.IMG_HEIGHT / 2

    def get_location(self, left_pixel, right_pixel_x):
        """
        Calculates 3D position.
        
        Args:
            left_pixel (tuple): (x, y) coordinates of object in Left Camera
            right_pixel_x (float): x coordinate of object in Right Camera
                                   (y is assumed same as left pixels)
        
        Returns:
            dict: Contains 'x', 'y', 'z', 'distance', 'angle_degrees'
        """
        u_L, v_L = left_pixel
        u_R = right_pixel_x

        disparity = u_L - u_R
        
        # Safety check for infinite distance
        if disparity <= 0:
            return None 

        # Calculate Depth
        # Formula: Z = (f * T) / disparity
        z = (self.FOCAL_LENGTH * self.BASELINE) / disparity

        # Calculate Horizontal (X)
        # Formula: X = (u_L - cx) * Z / f
        x = (u_L - self.cx) * z / self.FOCAL_LENGTH

        # Calculate Vertical (Y)
        # Formula: Y = (v_L - cy) * Z / f
        y = (v_L - self.cy) * z / self.FOCAL_LENGTH

        # Calculate Total Straight-Line Distance (Euclidean)
        total_distance = math.sqrt(x**2 + y**2 + z**2)

        # Calculate Direction (Angle relative to nose)
        # 0 degrees is straight ahead. Negative is Left, Positive is Right.
        angle_rad = math.atan2(x, z)
        angle_deg = math.degrees(angle_rad)

        return {
            "x": round(x, 3),            # All Meters
            "y": round(y, 3),            
            "z_depth": round(z, 3),     
            "distance": round(total_distance, 3), 
            "angle": round(angle_deg, 1)
        }

if __name__ == "__main__":
    calc = MiroDepthCalculator()

    # Example: Object detected at (400, 200) in Left Eye and (380, 200) in Right Eye
    obj_left_pixel = (400, 200) 
    obj_right_x = 380

    result = calc.get_location(obj_left_pixel, obj_right_x)

    if result:
        print(f"--- Object Location ---")
        print(f"Position (X,Y,Z): {result['x']}, {result['y']}, {result['z_depth']} meters")
        print(f"Total Distance:   {result['distance']} meters")
        print(f"Direction:        {result['angle']} degrees from center")
    else:
        print("Object is too far away or invalid disparity.")
