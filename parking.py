#-- coding:utf-8 --
import pygame
import math

class CarSimulator:
    def __init__(self):
        # Initialize Pygame
        pygame.init()

        # Set up the window
        self.screen_width = 1200
        self.screen_height = 850
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("Car Simulator")

        # set FPS
        self.clock = pygame.time.Clock()

        # Set colors
        self.background_color = (255, 255, 255)
        self.line_color = (0, 0, 0)
        self.tag_color = (0, 255, 0)
        self.parking_line_color = (255, 0, 0)
        self.sky_blue = (135, 206, 235)
        self.planning_color = (65, 105, 225)
        self.tracking_color = (240, 128, 128)

        # Tilted rectangle coordinates and dimensions
        self.parking_x = 1030
        self.parking_y = 45
        self.parking_width = 93
        self.parking_height = 132

        # Fill the screen with the background color
        self.screen.fill(self.background_color)

        # Define the points of the end line
        self.end_point1 = (1100, 0)
        self.end_point2 = (1200, 100)

        self.tag_point1 = (1100, 20)
        self.tag_point2 = (1172, 92)

        # Define the line thickness
        self.line_thickness = 28
        self.tag_thickness = 12

        # Draw a sky blue circle at (100, 350)
        self.circle_center1 = (100, 350)
        self.circle_center2 = (100, 750)
        self.circle_center3 = (500, 700)
        self.circle_center4 = (900, 700)
        self.circle_center5 = (1100, 300)
        self.circle_radius = 65

        # Create the Planning and Tracking rectangles
        self.planning_rect = pygame.Rect(250, 775, 200, 60)
        self.tracking_rect = pygame.Rect(500, 775, 200, 60)

        # Create font objects for the text
        self.font = pygame.font.SysFont(None, 40)

        # Create the text surfaces for Planning and Tracking
        self.planning_text = self.font.render("Planning", True, (255, 255, 255))
        self.tracking_text = self.font.render("Tracking", True, (255, 255, 255))

        # Create car image at the center of the first circle
        self.car = pygame.image.load("/home/gihong/car.png")
        self.car_speed = 0
        self.car_angle = 0
        self.car = pygame.transform.rotate(self.car, 45)
        self.car_rect = self.car.get_rect()
        self.car_rect.center = self.circle_center1

        # Flag to indicate if tracking is active
        self.tracking_active = False

        # Update the screen
        pygame.display.flip()

        # Main game loop
        self.running = True

    def run(self):
        while self.running:
            self.clock.tick(60)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    self.handle_mouse_click(event)
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_DELETE:
                    self.remove_car()

            if self.tracking_active:
                self.update_car_position()

            self.redraw_screen()

        pygame.quit()

    def handle_mouse_click(self, event):
        mouse_pos = pygame.mouse.get_pos()
        self.car_speed = 0
        self.car_angle = 0

        # Check if a circle was clicked
        circles = [self.circle_center1, self.circle_center2, self.circle_center3,
                   self.circle_center4, self.circle_center5]
        circle_radii = [self.circle_radius] * len(circles)

        for i in range(len(circles)):
            if pygame.Rect(circles[i][0] - circle_radii[i], circles[i][1] - circle_radii[i],
                           circle_radii[i] * 2, circle_radii[i] * 2).collidepoint(mouse_pos):
                self.create_car(i)
                break

        # Check if Planning or Tracking rectangles were clicked
        if self.planning_rect.collidepoint(mouse_pos):
            print("Planning")
        elif self.tracking_rect.collidepoint(mouse_pos):
            print("Tracking")
            self.tracking_active = True
            self.car_speed = 1
            self.car_angle = 1

    def create_car(self, circle_index):
        car_image = pygame.image.load("/home/gihong/car.png")
        angles = [45, 0, 90, 45, 45]
        self.car = pygame.transform.rotate(car_image, angles[circle_index])
        self.car_rect = self.car.get_rect()
        self.car_rect.center = [self.circle_center1, self.circle_center2,
                                self.circle_center3, self.circle_center4, self.circle_center5][circle_index]

    def remove_car(self):
        self.car = None
        self.car_speed = 0
        self.car_angle = 0

    def update_car_position(self):
        car_angle_rad = math.radians(self.car_angle)
        car_speed_x = self.car_speed * math.sin(car_angle_rad)
        car_speed_y = self.car_speed * math.cos(car_angle_rad)
        self.car_rect.x += car_speed_x
        self.car_rect.y -= car_speed_y

    def redraw_screen(self):
        self.screen.fill(self.background_color)
        pygame.draw.line(self.screen, self.line_color, self.end_point1, self.end_point2, self.line_thickness)
        pygame.draw.line(self.screen, self.tag_color, self.tag_point1, self.tag_point2, self.tag_thickness)
        pygame.draw.circle(self.screen, self.sky_blue, self.circle_center1, self.circle_radius)
        pygame.draw.circle(self.screen, self.sky_blue, self.circle_center2, self.circle_radius)
        pygame.draw.circle(self.screen, self.sky_blue, self.circle_center3, self.circle_radius)
        pygame.draw.circle(self.screen, self.sky_blue, self.circle_center4, self.circle_radius)
        pygame.draw.circle(self.screen, self.sky_blue, self.circle_center5, self.circle_radius)

        tilted_rect = pygame.Surface((self.parking_width, self.parking_height), pygame.SRCALPHA)
        pygame.draw.rect(tilted_rect, self.parking_line_color, (0, 0, self.parking_width, self.parking_height), 3)
        tilted_rect = pygame.transform.rotate(tilted_rect, 135)
        tilted_rect_rect = tilted_rect.get_rect(
            center=(self.parking_x + self.parking_width / 2, self.parking_y + self.parking_height / 2))
        self.screen.blit(tilted_rect, tilted_rect_rect)

        if self.car:
            self.screen.blit(self.car, self.car_rect)

        pygame.draw.rect(self.screen, self.planning_color, self.planning_rect)
        pygame.draw.rect(self.screen, self.tracking_color, self.tracking_rect)

        planning_text_rect = self.planning_text.get_rect(center=self.planning_rect.center)
        tracking_text_rect = self.tracking_text.get_rect(center=self.tracking_rect.center)
        self.screen.blit(self.planning_text, planning_text_rect)
        self.screen.blit(self.tracking_text, tracking_text_rect)

        pygame.display.flip()

if __name__ == "__main__":
    simulator = CarSimulator()
    simulator.run()
