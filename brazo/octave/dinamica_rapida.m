function [M, h, T_tcp, Jg] = dinamica_rapida(q, qd, D)
  % DINAMICA_RAPIDA  M(q) y h(q,qd) = c(q,qd) + g(q) en una sola pasada (para simular y verificar).
  %   M  : Lagrange con jacobianos de centros de masa, M = sum m Jv'Jv + Jw' I Jw + diag(N^2 Jm)
  %   h  : Newton-Euler recursivo con qdd = 0 y gravedad (da c + g)
  %   T_tcp, Jg : pose del TCP y jacobiano geometrico (de regalo, misma cinematica)
  % Misma formulacion que rne_brazo/lagrange_brazo, pero sin llamadas a funciones por producto
  % vectorial ni recomputo de la cinematica: ~10x mas rapida en Octave. La equivalencia se
  % comprueba en verificar_trayectoria_brazo.m.
  dh = D.dh;
  R = zeros(3,3,7); o = zeros(3,7); R(:,:,1) = eye(3);
  for i = 1:6
    th = q(i) + dh(i,1); ct = cos(th); st = sin(th); ca = cos(dh(i,4)); sa = sin(dh(i,4));
    Ri = [ct, -st*ca, st*sa; st, ct*ca, -ct*sa; 0, sa, ca];
    R(:,:,i+1) = R(:,:,i) * Ri;
    o(:,i+1) = o(:,i) + R(:,:,i) * [dh(i,3)*ct; dh(i,3)*st; dh(i,2)];
  end
  z = squeeze(R(:,3,1:6));                    % eje de la articulacion i = z_{i-1}
  p = o(:,7) + D.ht * R(:,3,7);               % TCP
  c = zeros(3,6); I = zeros(3,3,6);
  for i = 1:6
    c(:,i) = o(:,i+1) + R(:,:,i+1) * D.rc(:,i);
    I(:,:,i) = R(:,:,i+1) * D.Ic(:,:,i) * R(:,:,i+1)';
  end

  % ---- M por jacobianos de los centros de masa
  M = diag(D.Jref);
  for k = 1:6
    Jv = zeros(3,6); Jw = zeros(3,6);
    for j = 1:k
      a = z(:,j); b = c(:,k) - o(:,j);
      Jv(:,j) = [a(2)*b(3)-a(3)*b(2); a(3)*b(1)-a(1)*b(3); a(1)*b(2)-a(2)*b(1)];
      Jw(:,j) = a;
    end
    M = M + D.m(k) * (Jv' * Jv) + Jw' * I(:,:,k) * Jw;
  end

  % ---- h = c + g por Newton-Euler (qdd = 0)
  w = zeros(3,1); al = zeros(3,1); aP = zeros(3,1);
  F = zeros(3,6); Nm = zeros(3,6); Pp = [o(:,1:6), p];
  for i = 1:6
    zi = z(:,i); wp = w; wq = zi * qd(i);
    w = wp + wq;
    al = al + [wp(2)*wq(3)-wp(3)*wq(2); wp(3)*wq(1)-wp(1)*wq(3); wp(1)*wq(2)-wp(2)*wq(1)];
    r1 = c(:,i) - Pp(:,i); r2 = Pp(:,i+1) - Pp(:,i);
    t1 = [al(2)*r1(3)-al(3)*r1(2); al(3)*r1(1)-al(1)*r1(3); al(1)*r1(2)-al(2)*r1(1)];
    u1 = [w(2)*r1(3)-w(3)*r1(2); w(3)*r1(1)-w(1)*r1(3); w(1)*r1(2)-w(2)*r1(1)];
    v1 = [w(2)*u1(3)-w(3)*u1(2); w(3)*u1(1)-w(1)*u1(3); w(1)*u1(2)-w(2)*u1(1)];
    ac = aP + t1 + v1;
    t2 = [al(2)*r2(3)-al(3)*r2(2); al(3)*r2(1)-al(1)*r2(3); al(1)*r2(2)-al(2)*r2(1)];
    u2 = [w(2)*r2(3)-w(3)*r2(2); w(3)*r2(1)-w(1)*r2(3); w(1)*r2(2)-w(2)*r2(1)];
    v2 = [w(2)*u2(3)-w(3)*u2(2); w(3)*u2(1)-w(1)*u2(3); w(1)*u2(2)-w(2)*u2(1)];
    aP = aP + t2 + v2;
    F(:,i) = D.m(i) * (ac - D.g0);
    Iw = I(:,:,i) * w;
    Nm(:,i) = I(:,:,i) * al + [w(2)*Iw(3)-w(3)*Iw(2); w(3)*Iw(1)-w(1)*Iw(3); w(1)*Iw(2)-w(2)*Iw(1)];
  end
  f = zeros(3,1); n = zeros(3,1); h = zeros(6,1);
  for i = 6:-1:1
    r1 = c(:,i) - Pp(:,i); r2 = Pp(:,i+1) - Pp(:,i); Fi = F(:,i);
    n = Nm(:,i) + n + [r1(2)*Fi(3)-r1(3)*Fi(2); r1(3)*Fi(1)-r1(1)*Fi(3); r1(1)*Fi(2)-r1(2)*Fi(1)] ...
                    + [r2(2)*f(3)-r2(3)*f(2); r2(3)*f(1)-r2(1)*f(3); r2(1)*f(2)-r2(2)*f(1)];
    f = Fi + f;
    h(i) = z(:,i)' * n;
  end

  if nargout > 2
    T_tcp = [R(:,:,7), p; 0 0 0 1];
  end
  if nargout > 3
    Jg = zeros(6,6);
    for j = 1:6
      a = z(:,j); b = p - o(:,j);
      Jg(:,j) = [a(2)*b(3)-a(3)*b(2); a(3)*b(1)-a(1)*b(3); a(1)*b(2)-a(2)*b(1); a];
    end
  end
end
