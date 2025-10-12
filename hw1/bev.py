import cv2
import numpy as np

points = []

class Projection(object):

    def __init__(self, image_path, points):
        """
            :param points: Selected pixels on top view(BEV) image
        """

        if type(image_path) != str:
            self.image = image_path
        else:
            self.image = cv2.imread(image_path)
        self.height, self.width, self.channels = self.image.shape

    def top_to_front(self, theta=0, phi=0, gamma=0, dx=0, dy=0, dz=0, fov=90):
        """
            Project the top view pixels to the front view pixels.
            :return: New pixels on perspective(front) view image
        """
        # Camera Intrinsic Parameters based on spec (512x512, 90deg FOV) [cite: 37, 38]
        img_width, img_height = 512, 512
        cx, cy = img_width / 2, img_height / 2
        fx = fy = cx / np.tan(np.deg2rad(fov / 2))
        K = np.array([[fx, 0, cx],
                      [0, fy, cy],
                      [0, 0, 1]])
        K_inv = np.linalg.inv(K)

        # Camera Extrinsic Parameters from spec [cite: 27, 28]
        # Front Camera (Cam1) Pose (Camera to World)
        R_front = np.identity(3)
        t_front = np.array([[0], [1], [0]])

        # BEV Camera (Cam2) Pose (Camera to World)
        bev_pitch_angle = -np.pi / 2
        R_bev = np.array([[1, 0, 0],
                          [0, np.cos(bev_pitch_angle), -np.sin(bev_pitch_angle)],
                          [0, np.sin(bev_pitch_angle), np.cos(bev_pitch_angle)]])
        t_bev = np.array([[0], [2.5], [0]])

        # Create World-to-Front-Camera Transformation Matrix
        T_world_front = np.hstack((R_front, t_front))
        T_world_front = np.vstack((T_world_front, [0, 0, 0, 1]))
        T_front_world = np.linalg.inv(T_world_front)
        
        new_pixels = []
        for p_bev in self.points:
            # 1. Unproject the 2D BEV pixel to a 3D ray in the BEV camera's coordinate system.
            p_bev_homogeneous = np.array([p_bev[0], p_bev[1], 1.0])
            ray_cam_bev = K_inv @ p_bev_homogeneous

            # 2. Transform the ray from BEV camera coordinates to world coordinates.
            ray_world = R_bev @ ray_cam_bev
            
            # 3. Calculate the intersection of the ray with the ground plane (Y=0) in world coordinates.
            # The ray originates from the BEV camera's position (t_bev).
            # Ray equation: P(t) = t_bev + t * ray_world
            # At Y=0: t_bev[1] + t * ray_world[1] = 0
            if ray_world[1] >= 0: continue # Ray is not pointing towards the ground plane
            
            t = -t_bev[1] / ray_world[1]
            P_world = t_bev.flatten() + t * ray_world
            P_world_homogeneous = np.append(P_world, 1)

            # 4. Transform the 3D world point into the front camera's coordinate system.
            P_cam_front_homogeneous = T_front_world @ P_world_homogeneous
            
            # 5. Project the 3D point onto the front camera's 2D image plane.
            # Perform perspective division
            P_cam_front = P_cam_front_homogeneous[:3] / P_cam_front_homogeneous[2]
            p_front_homogeneous = K @ P_cam_front
            
            u, v = int(p_front_homogeneous[0]), int(p_front_homogeneous[1])

            if 0 <= u < img_width and 0 <= v < img_height:
                new_pixels.append([u, v])
        return new_pixels

    def show_image(self, new_pixels, img_name='projection.png', color=(0, 0, 255), alpha=0.4):
        """
            Show the projection result and fill the selected area on perspective(front) view image.
        """

        new_image = cv2.fillPoly(
            self.image.copy(), [np.array(new_pixels)], color)
        new_image = cv2.addWeighted(
            new_image, alpha, self.image, (1 - alpha), 0)

        cv2.imshow(
            f'Top to front view projection {img_name}', new_image)
        cv2.imwrite(img_name, new_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return new_image


def click_event(event, x, y, flags, params):
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:

        print(x, ' ', y)
        points.append([x, y])
        font = cv2.FONT_HERSHEY_SIMPLEX
        # cv2.putText(img, str(x) + ',' + str(y), (x+5, y+5), font, 0.5, (0, 0, 255), 1)
        cv2.circle(img, (x, y), 3, (0, 0, 255), -1)
        cv2.imshow('image', img)

    # checking for right mouse clicks
    if event == cv2.EVENT_RBUTTONDOWN:

        print(x, ' ', y)
        font = cv2.FONT_HERSHEY_SIMPLEX
        b = img[y, x, 0]
        g = img[y, x, 1]
        r = img[y, x, 2]
        # cv2.putText(img, str(b) + ',' + str(g) + ',' + str(r), (x, y), font, 1, (255, 255, 0), 2)
        cv2.imshow('image', img)


if __name__ == "__main__":

    pitch_ang = -90

    front_rgb = "bev_data/front1.png"
    top_rgb = "bev_data/bev1.png"

    # click the pixels on window
    img = cv2.imread(top_rgb, 1)
    cv2.imshow('image', img)
    cv2.setMouseCallback('image', click_event)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    projection = Projection(front_rgb, points)
    new_pixels = projection.top_to_front(theta=pitch_ang)
    projection.show_image(new_pixels)
