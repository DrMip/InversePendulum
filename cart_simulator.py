import pygame
import sys
import math

WHITE = (255, 255, 255)
BLUE = (50, 100, 200)
BLACK = (0, 0, 0)
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

class Visualize:

    def __init__(self):
        # Initialize Pygame
        pygame.init()

        # Screen settings
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Cart Movement Example")

        # Colors


        # Cart settings
        self.cart_width = 100
        self.cart_height = 40
        self.cart_x = (SCREEN_WIDTH - self.cart_width) // 2
        self.cart_y = SCREEN_HEIGHT - 100
        self.cart_speed = 5

        # Load cart image
        self.cart_image = pygame.image.load("cart.png")
        self.cart_image = pygame.transform.scale(self.cart_image, (200, 200))
        self.cart_rect = self.cart_image.get_rect()
        self.cart_rect.midbottom = (SCREEN_WIDTH // 2, SCREEN_HEIGHT - 200)

        # Load rod image (pointing up)
        self.rod_image = pygame.image.load("rod.png")
        self.rod_image = pygame.transform.scale(self.rod_image, (200, 200))
        self.rod_original = self.rod_image  # Keep the original for clean rotations
        self.rod_rect = self.rod_image.get_rect()
        self.rod_origin = (self.rod_image.get_width() // 2, self.rod_image.get_height())  # bottom center

        # Rod settings
        self.rod_angle = 0

        # Clock
        self.clock = pygame.time.Clock()
    def play(self):
        # Main game loop
        running = True
        while running:
            self.screen.fill(WHITE)

            # Event handling
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            keys = pygame.key.get_pressed()
            if keys[pygame.K_LEFT]:
                self.cart_rect.x -= self.cart_speed
            if keys[pygame.K_RIGHT]:
                self.cart_rect.x += self.cart_speed

            # Control the angle of the rod
            if keys[pygame.K_UP]:
                self.rod_angle -= 1
            if keys[pygame.K_DOWN]:
                self.rod_angle += 1


            # Boundary check
            self.cart_rect.x = max(0, min(SCREEN_WIDTH - self.cart_rect.width, self.cart_rect.x))

            # Draw cart image
            self.screen.blit(self.cart_image, self.cart_rect)
            pygame.draw.line(self.screen, BLACK, (0,SCREEN_HEIGHT - 220), (SCREEN_WIDTH, SCREEN_HEIGHT - 220), 10)

            # Rotate the rod image
            self.rotate_pendulum(math.radians(self.rod_angle))


            # Update display
            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()
        sys.exit()

    def simulate(self, location : float, theta : float):
        self.screen.fill(WHITE)

        self.cart_rect.x = location * SCREEN_WIDTH / 5
        # Boundary check
        self.cart_rect.x = max(0, min(SCREEN_WIDTH - self.cart_rect.width, int(self.cart_rect.x)))

        # Draw cart image
        self.screen.blit(self.cart_image, self.cart_rect)
        pygame.draw.line(self.screen, BLACK, (0, SCREEN_HEIGHT - 220), (SCREEN_WIDTH, SCREEN_HEIGHT - 220), 10)

        self.rotate_pendulum(theta)

        # Update display
        pygame.display.flip()
        self.clock.tick(60)

    def rotate_pendulum(self, theta):
        # Rotate the rod image
        rotated_rod = pygame.transform.rotate(self.rod_original, -math.degrees(theta))
        rotated_rect = rotated_rod.get_rect()

        # Calculate new blit position so bottom center stays attached to cart top
        pivot_x = self.cart_rect.centerx
        pivot_y = self.cart_rect.top + 70

        # Offset from image center to pivot point (in original image)
        offset_x = self.rod_origin[0] - self.rod_original.get_width() / 2
        offset_y = self.rod_origin[1] - self.rod_original.get_height() / 2

        # Rotate that offset
        angle_rad = math.radians(self.rod_angle)
        rot_offset_x = offset_x * math.cos(angle_rad) - offset_y * math.sin(angle_rad)
        rot_offset_y = offset_x * math.sin(angle_rad) + offset_y * math.cos(angle_rad)

        # Final blit position
        blit_x = pivot_x - rotated_rect.width / 2 - rot_offset_x
        blit_y = pivot_y - rotated_rect.height / 2 - rot_offset_y

        # Draw the rotated rod
        self.screen.blit(rotated_rod, (blit_x, blit_y))

if __name__ == "__main__":
    vis = Visualize()

    vis.play()