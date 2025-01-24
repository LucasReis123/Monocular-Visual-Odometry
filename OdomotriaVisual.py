import cv2
import numpy as np
import matplotlib.pyplot as plt
import random

class VisualOdometry():
    def __init__(self, qnt):
        super().__init__()

        self.K, self.P = self.K_P_matriz()
        
        # Inicializa o detector SIFT
        self.sift = cv2.SIFT_create()
        # Inicializa a coorespondencia de pontos de força bruta
        self.bf = cv2.BFMatcher()

        self.last_keypoints = None
        self.last_descriptors = None

        self.X_state = np.array([[0],
                                 [0],
                                 [0],
                                 [0],
                                 [0],
                                 [0]], dtype=float)
        
        # criando um contador para o plt
        self.cont = 0

        self.fig_2d, self.ax_2d = plt.subplots()
        self.ax_2d.set_xlim(-25, 200)
        self.ax_2d.set_xlim(-25, 200)
        self.ax_2d.set_title("Trajetória do carro")
        self.ax_2d.set_xlabel("X")
        self.ax_2d.set_ylabel("Y")
        self.ax_2d.grid(True)


        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim(-25, 200)
        self.ax.set_xlim(-25, 200)
        self.ax.set_zlim(-20, 20)
        self.ax.set_title("Trajetória do carro")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

        self.ground_truth(qnt)
        # Adiciona rótulos personalizados com cores
        legend_labels = [
            plt.Line2D([0], [0], marker='o', color='red', label='Ground Truth', markersize=10, markerfacecolor='red'),
            plt.Line2D([0], [0], marker='o', color='green', label='Odometry', markersize=10, markerfacecolor='green')
        ]

        # Cria a legenda com os rótulos personalizados
        self.ax.legend(handles=legend_labels, loc='upper left')
        
        self.fig_image, self.ax_image = plt.subplots()
        self.ax_image.set_title("Carro")

        
        plt.ion()

    def ground_truth(self, qnt):
        with open("./data_odometry_poses/dataset/poses/00.txt", 'r') as f:
            for i, line in enumerate(f):
                if i < qnt:
                    P = np.fromstring(line, dtype=np.float64, sep=" ")
                    P = np.reshape(P, (3, 4))
                    T = P[:, 3]
                    
                    x = T[0]
                    y = T[2]
                    z = T[1]
                    self.ax.plot(x, y, z, 'ro')
                    self.ax_2d.plot(x, y, 'ro')
                
                else:
                    break


    def K_P_matriz(self):
        with open("./Sequences/00/calib.txt", 'r') as f:
            for line in f:
                if line.startswith("P0:"):
                    num = line.split(": ")[1]
                    P = np.fromstring(num, dtype=np.float64, sep=" ")
                    P = np.reshape(P, (3, 4))
                    K = P[:3, :3]
                    
                    return K, P

    def best_matches(self, matches, qnt, interval):
        cont = 0
        r = 0.5

        good_matches = []

        while cont < 50:
            cont+=1
            del good_matches[:]

            # Filtrando as correspondencias usando o teste de razão de Lowe
            for m, n in matches:
                if m.distance < r * n.distance:
                    good_matches.append(m)

            len_good_m = len(good_matches)

            if len_good_m - interval <= qnt and len_good_m + interval >= qnt:

                return good_matches
            elif len_good_m - interval > qnt:
                r = r - (1 -r)/2
            elif len_good_m + interval < qnt:
                r = r + (1 -r)/2
        
        good_matches = [m for m, n in matches if m.distance < 0.75 * n.distance]

        return good_matches

    def sift_matches(self, keypoints, descriptors):
        matches = self.bf.knnMatch(self.last_descriptors, descriptors, k=2)

        
        good_matches = self.best_matches(matches, 200, 20)

        # Pegando os pontos que caracterizam correspondencia e agrupando
        correspondents = []
        for match in good_matches:
            img1_idx = match.queryIdx
            img2_idx = match.trainIdx

            # Pegando as coordenadas dos pontos correspondentes
            (x1, y1) = self.last_keypoints[img1_idx].pt
            (x2, y2) = keypoints[img2_idx].pt

            correspondents.append(((x1, y1),(x2, y2)))


        return correspondents

    def calculation_of_the_F(self, points):
        # Calculando a matriz A para aplicar decomposição SVD
        A = np.array([[x2*x1, x2*y1, x2, x1*y2, y2*y1, y2, x1, y1, 1]
                        for (x1, y1), (x2, y2) in points])

        # utilizando SVD para resolver Af = 0
        U, S, Vt = np.linalg.svd(A)
        F = Vt[-1].reshape(3, 3)

        return F

    def other_points(self, F, correspondents):
        points = []
        for (x1, y1), (x2, y2) in correspondents:
            X = np.array([x1, y1, 1]).T
            Xt = np.array([x2, y2, 1])
            
            # Calculando a resposta da matriz F para todos os pontos:
            R = np.abs(Xt @ F @ X)
            if R < 0.1:
                points.append(((x1, y1), (x2, y2)))

        return points


    def F_RANSAC(self, correspondents, qnt=15):
        len_better_points = 0
        better_points = None

        for _ in range(qnt):
            # Pegando 10 correspondencia aleatorias "qnt" vezes
            random_points = random.sample(correspondents, 15)
            #random_points = self.randomly_points(correspondents)

            F = self.calculation_of_the_F(random_points)
            
            all_points = self.other_points(F, correspondents)
            len_all_points = len(all_points)

            if len_all_points > len_better_points:
                len_better_points = len_all_points
                better_points = all_points

        F = self.calculation_of_the_F(better_points)
        #(len_better_points)
        return F, better_points

    def points_that_form_F(self, points):
        q1 = np.empty((3,0))
        q2 = np.empty((3,0))

        for Q1, Q2 in points:
            x1, y1 = Q1
            x2, y2 = Q2

            X1 = np.array([x1, y1, 1]).reshape(3,1)
            X2 = np.array([x2, y2, 1]).reshape(3,1)

            q1 = np.hstack([q1, X1])
            q2 = np.hstack([q2, X2])

        return q1, q2

    def essential_matriz(self, F):

        # Como estamos utilizando a mesma camera, a matriz de parametros internos é a mesma
        Kt = self.K.T

        E = Kt @ F @ self.K
        
        return E
    
    def decomposition_E(self, E):
        # Utilizando decomposicao SVD
        U, S, Vt = np.linalg.svd(E)

        # Garantindo que o determinante seja positivo
        if np.linalg.det(U) < 0:
            U *= -1
        if np.linalg.det(Vt) < 0:
            Vt *= -1

        # Calculando as possiveis rotacoes e translacoes
        W = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        R1 = np.array(U @ W @ Vt)
        R2 = np.array(U @ W.T @ Vt)
        t1 = np.array(U[:, 2])
        t1.reshape((3, 1))
        t2 = - t1

        return R1, R2, t1, t2
    
    def triangulate_points(self, P2, q1, q2):
        # Pegando os vetores transpostos da matriz de projecao 1:
        P11T = self.P[0, :]
        P12T = self.P[1, :]
        P13T = self.P[2, :]

        # Pegando os vetores transpostos da matriz de projecao 1:
        P21T = P2[0, :]
        P22T = P2[1, :]
        P23T = P2[2, :]

        # Pegando o numero de pontos
        num_points = q1.shape[1]

        # Iniciando a matriz que vai armazenar os pontos 3D do primeiro frame
        Q1_3D = np.zeros((4, num_points))

        for i in range(num_points):
            # Pegando os pontos 2D em coordenadas homogeneas
            x1, y1, _ = q1[:, i]
            x2, y2, _ = q2[:, i]

            A = np.array([(x1*P13T - P11T),
                          (y1*P13T - P12T),
                          (x2*P23T - P21T),
                          (y2*P23T - P22T)])
            
            # Utilizando decomposicao SVD para achar o ponto 3D
            _, _, Vt = np.linalg.svd(A)
            Vt = Vt[-1, :].reshape((4,1))

            Q1_3D[:, i:i+1] = Vt

        # Normalizando as coordenadas 3D
        Q1_3D /= Q1_3D[3, :]
        
        return Q1_3D

    def rot_trans(self, R1, R2, t1, t2, q1, q2):
        # Pegando todas as combinacoes
        combinations = [[R1, t1.T], [R2, t1.T], [R1, t2.T], [R2, t2.T]]
        Z_scale = []

        for comb in combinations:
            aux = []
            R, t = comb
            T = np.hstack([R, t.reshape(-1, 1)])
            
            # Calculando a matriz de projecao do segundo frame 
            P2 = self.K @ T

            # Triangulando os pontos correspondentes para achar o ponto 3D visto no priemrio frame
            Q1 = self.triangulate_points(P2, q1, q2)

            # Aplicando a matriz de rotacao e translacao para ver a projecao de Q2
            Q2 =  T @ Q1

            # Retirando a homogeneizacao dos vetores
            Q1 = Q1[:3, :]
            Q2 = Q2[:3, :]

            # Encontrando o numero de pontos em que z é positivo
            z_sum = sum(Q1[2, :] > 0) + sum (Q2[2, :] > 0)

            # Estimando a escala relativa baseada na relacao entre a norma da fireneca entre 2 pontos 3D consecutivos
            scale = np.mean(np.linalg.norm(Q1.T[:-1] - Q1.T[1:]) / np.linalg.norm(Q2.T[:-1] - Q2.T[1:]))

            # Salvando as informacoes para pegar a melhor depois
            aux.append(z_sum)
            aux.append(scale)
            Z_scale.append(aux)

        # Escolhendo a melhor combinacao com base na quantidade de z positivos
        better_index = np.argmax([z[0] for z in Z_scale])
        R, t = combinations[better_index]
        scale = Z_scale[better_index][1]
        t = t * scale

        return R, t
    
    def rotation_matrix_to_euler_angles(self, R):
        sy = np.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])

        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            x = np.arctan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])
    
    def X_state_update(self, theta, t):

        # Criando o vetor de translacao que indica a nova translacao
        T = np.array([[-t[0]],
                      [-t[2]],
                      [-t[1]]])
        
        # Calculando a matriz de rotacao 3D que indica a rotacao dos planos seguindo a ordem: RxRyRz
        self.X_state[3] += theta[0]
        self.X_state[4] += theta[2]
        self.X_state[5] += theta[1]
        
        tx = self.X_state[3, 0]
        ty = self.X_state[4, 0]
        tz = self.X_state[5, 0]

        # Calculando os indices da matriz
        A = np.cos(tx)
        a = np.sin(tx)
        B = np.cos(ty)
        b = np.sin(ty)
        C = np.cos(tz)
        c = np.sin(tz)


        R = np.array([(B*C, -B*c, b),
                      (a*b*C + A*c, A*C - a*b*c, -a*B),
                      (a*c - A*b*C, A*b*c + a*C, A*B)])
        
        new_T = R @ T

        self.X_state[:3] += new_T

    def update_trajectory(self):
        x = self.X_state[0, 0]
        y = self.X_state[1, 0]
        z = self.X_state[2, 0]

        self.ax.plot(x, y, z, 'go') # Ponto 3d
        
        self.ax_2d.plot(x, y, 'go') # Ponto 2D

    def draw_corners(self, image, corners):
        for x, y in corners:
            cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
        return image

    def Movement(self, I):
        keypoints, descriptors = self.sift.detectAndCompute(I, None)

        if self.last_keypoints is None:
            # Como esse é o primeiro frame, não tem como comparar
            # Pegando os dados do primeiro frame
            self.last_keypoints = keypoints
            self.last_descriptors = descriptors
            return
        
        # Calculando as correspondencias
        correspondents = self.sift_matches(keypoints, descriptors)
        
        # Considerando um numero minimo de correspondencias
        if len(correspondents) < 15:
            print(len(correspondents))
            self.last_keypoints = keypoints
            self.last_descriptors = descriptors
            return


        # Obtendo a matriz fundamental e os pontos que formam ela
        F, points = self.F_RANSAC(correspondents)


        # Obtendo os pontos que formam F em coordenadas homogêneas
        q1, q2 = self.points_that_form_F(points)

        # Calculando a matriz essencial
        E = self.essential_matriz(F)
        
        # Descobrindo as possiveis rotacoes e translacoes
        R1, R2, t1, t2 = self.decomposition_E(E)

        # Obtendo a combinacao correta
        R, t = self.rot_trans(R1, R2, t1, t2, q1, q2)
        #print(R)
        #print(t)

        # Decompondo a matriz de rotacao em variacao dos angulos
        theta = self.rotation_matrix_to_euler_angles(R)

        # Atualizando o vetor de estado
        self.X_state_update(theta, t)

        # Atualizando a trajetoria no grafico
        self.update_trajectory()

        
        if self.cont % 5 == 0:
            self.cont += 1

            plt.figure(self.fig_2d.number)
            plt.draw()


            plt.figure(self.fig.number)
            elev = 60  # Elevação em graus
            azim = 210  # Azimute em graus

            self.ax.view_init(elev=elev, azim=azim)

            corners = [(int(x), int(y)) for _, (x, y) in correspondents]
            I = self.draw_corners(I, corners)
            image_plot = self.ax_image.imshow(I, cmap='gray')
            plt.figure(self.fig_image.number)
            image_plot.set_data(I)
            plt.draw()

            plt.pause(0.01)
        else:
            self.cont += 1
        

        self.last_keypoints = keypoints
        self.last_descriptors = descriptors


# ---------------------------------- MAIN ----------------------------------------

def main(args=None):
    qnt = 270
    Odometry = VisualOdometry(qnt)
    
    for i in range(qnt):
        fname = f"./Sequences/00/image_0/{i:06d}.png"
        gray = cv2.imread(fname, cv2.IMREAD_GRAYSCALE)

        Odometry.Movement(gray)



if __name__ == '__main__':
    main()