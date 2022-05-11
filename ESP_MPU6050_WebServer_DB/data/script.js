
let scene, camera, rendered, cube;

function timedRefresh(timeoutPeriod) {
  setTimeout("location.reload(true);",timeoutPeriod);
}

function parentWidth(elem) {
  return elem.parentElement.clientWidth;
}

function parentHeight(elem) {
  return elem.parentElement.clientHeight;
}

function init3D(){
  scene = new THREE.Scene();
  scene2 = new THREE.Scene();

  scene.background = new THREE.Color(0xffffff);
  scene2.background = new THREE.Color(0xffffff);


  camera = new THREE.PerspectiveCamera(10, parentWidth(document.getElementById("3Dcube1")) / parentHeight(document.getElementById("3Dcube1")), 0.1, 1000);
  camera2 = new THREE.PerspectiveCamera(10, parentWidth(document.getElementById("3Dcube2")) / parentHeight(document.getElementById("3Dcube2")), 0.1, 1000);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(parentWidth(document.getElementById("3Dcube1")), parentHeight(document.getElementById("3Dcube1")));
  renderer2 = new THREE.WebGLRenderer({ antialias: true });
  renderer2.setSize(parentWidth(document.getElementById("3Dcube2")), parentHeight(document.getElementById("3Dcube2")));

  document.getElementById('3Dcube1').appendChild(renderer.domElement);
  document.getElementById('3Dcube2').appendChild(renderer2.domElement);

  // Create a geometry
  const geometry = new THREE.BoxGeometry(5, 1, 1);

  // Materials of each face
  var cubeMaterials = [
    new THREE.MeshBasicMaterial({color:0x03045e}),
    new THREE.MeshBasicMaterial({color:0x5E0304}),
    new THREE.MeshBasicMaterial({color:0x045E03}),
    new THREE.MeshBasicMaterial({color:0x30035E}),
    new THREE.MeshBasicMaterial({color:0x5E5D03}),
    new THREE.MeshBasicMaterial({color:0x5E3003}),
  ];

  const material = new THREE.MeshFaceMaterial(cubeMaterials);

  cube = new THREE.Mesh(geometry, material);
  cube2 = new THREE.Mesh(geometry, material);
  scene.add(cube);
  scene2.add(cube2);
  camera.position.z = 30;
  camera2.position.z = 30;
  cube.rotation.y = 10;
  cube2.rotation.y = 10;
  renderer.render(scene, camera);
  renderer2.render(scene2, camera2);
}

  var scene3 = new THREE.Scene();
  var camera3 = new THREE.PerspectiveCamera(60, parentWidth(document.getElementById("3Dcube3")) / parentHeight(document.getElementById("3Dcube3")), 1, 1000);
  camera3.position.set(1, 5, 10);
  camera3.lookAt(scene3.position);
  var renderer3 = new THREE.WebGLRenderer({antialias: true});
  renderer3.setSize(parentWidth(document.getElementById("3Dcube3")), parentHeight(document.getElementById("3Dcube3")));
  document.getElementById('3Dcube3').appendChild(renderer3.domElement);

  scene3.add(new THREE.GridHelper(10, 10));

  var planeGeom = new THREE.PlaneGeometry(5, 5);
  planeGeom.rotateZ(Math.PI * 0.25);
  planeGeom.vertices[0].basePosition = new THREE.Vector3().copy(planeGeom.vertices[0]);
  planeGeom.vertices[2].set(0, 0, 0); // let's make a triangle from the plane

  var plane = new THREE.Mesh(planeGeom, new THREE.MeshBasicMaterial({
    color: "aqua",
    wireframe: true
  }));
  scene3.add(plane);

  var axis = new THREE.Vector3(0, 1, 0); // in three.js, up is positive Y


function render() {
  requestAnimationFrame(render);
  planeGeom.vertices[0].copy(planeGeom.vertices[0].basePosition).applyAxisAngle(axis, cube2.rotation.x / 100); // we'll use .applyAxisAngle() method
  planeGeom.verticesNeedUpdate = true;
  renderer3.render(scene3, camera3);
}

// Resize the 3D object when the browser window changes size
function onWindowResize(){
  camera.aspect = parentWidth(document.getElementById("3Dcube1")) / parentHeight(document.getElementById("3Dcube1"));
  //camera.aspect = window.innerWidth /  window.innerHeight;
  camera.updateProjectionMatrix();
  //renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setSize(parentWidth(document.getElementById("3Dcube1")), parentHeight(document.getElementById("3Dcube1")));

  camera2.aspect = parentWidth(document.getElementById("3Dcube2")) / parentHeight(document.getElementById("3Dcube2"));
  //camera2.aspect = window.innerWidth /  window.innerHeight;
  camera2.updateProjectionMatrix();
  //renderer.setSize(window.innerWidth, window.innerHeight);
  renderer2.setSize(parentWidth(document.getElementById("3Dcube2")), parentHeight(document.getElementById("3Dcube2")));

}

window.addEventListener('resize', onWindowResize, false);

// Create the 3D representation
init3D();
render();

//window.onload = timedRefresh(3000);

// Create events for the sensor readings
if (!!window.EventSource) {
  var source = new EventSource('/events');

  source.addEventListener('open', function(e) {
      console.log(e)
      console.log(e.target.readyState)
    console.log("Events Connected");
  }, false);

  source.addEventListener('error', function(e) {
    if (e.target.readyState != EventSource.OPEN) {
      console.log(e)
      console.log(e.target.readyState)
      console.log("Events Disconnected");
    }
  }, false);

  source.addEventListener('gyro_readings', function(e) {
    console.log("gyro_readings", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("gyroX").innerHTML = obj.rotationX_1;
    document.getElementById("gyroY").innerHTML = obj.rotationY_1;
    document.getElementById("gyroX2").innerHTML = obj.rotationX_2;
    document.getElementById("gyroY2").innerHTML = obj.rotationY_2;


    // Change cube rotation after receiving the readinds
    cube.rotation.x = obj.rotationY_1;
    cube.rotation.z = obj.rotationX_1;
    renderer.render(scene, camera);

    cube2.rotation.x = obj.rotationY_2;
    cube2.rotation.z = obj.rotationX_2;
    renderer2.render(scene2, camera2);

  }, false);
}