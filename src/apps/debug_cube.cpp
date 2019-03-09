// Copyright Pavlo 2018

#include <iostream>
#include <memory>
#include <iomanip>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "cv_gl/camera.h"
#include "cv_gl/mesh.h"
#include "cv_gl/shader.h"
#include "cv_gl/utils.h"

#include "cv_gl/gl_window.h"

#include "cv_gl/renderer.hpp"
#include "cv_gl/dobject.hpp"
#include "cv_gl/object_factory.hpp"

const int kWindowWidth = 1226 / 2;
const int kWindowHeight = 1028 / 2;

int main(int argc, char *argv[])
{

    std::shared_ptr<Camera> camera =
        std::make_shared<Camera>(glm::vec3(-3.0f, 0.0f, 1.5f));

    std::cout << "Hello debug cube!" << std::endl;

    GLWindow gl_window("OpenGL: Edit Space", kWindowWidth, kWindowHeight);
    gl_window.SetCamera(camera);

    // Renderer
    std::unique_ptr<Renderer> renderer(new Renderer(camera));

    std::shared_ptr<ColorObject> floor_obj(ObjectFactory::CreateFloor(1.0, 50));

    std::shared_ptr<ColorObject> camera_obj(
        ObjectFactory::CreateCameraFrustum());
    camera_obj->SetTranslation(glm::vec3(3.0f, 0.0f, 0.0f));

    std::shared_ptr<ColorObject> zero_cube_obj(
        ObjectFactory::CreateCube());
    zero_cube_obj->SetTranslation(glm::vec3(3.0f, -5.0f, 0.0f));

    std::shared_ptr<DObject> axes_obj(ObjectFactory::CreateAxes(1.0f));

    // std::cout << "Loading debug cube ..." << std::endl;
    // std::cin.ignore();
    std::shared_ptr<ModelObject> debug_cube_obj(
        ObjectFactory::CreateModelObject(
            "../data/objects/debug_cube/debug_cube.obj"));
    //   debug_cube_obj->SetScale(glm::vec3(0.2f));
    debug_cube_obj->SetTranslation(glm::vec3(0.0f, 0.0f, 3.0f));

    std::cout << "Loading nanosuit ..." << std::endl;
    // std::cin.ignore();
    std::shared_ptr<ModelObject> nanosuit_obj(
        ObjectFactory::CreateModelObject(
            "../data/objects/nanosuit/nanosuit.obj"));
    nanosuit_obj->SetScale(glm::vec3(0.2f));
    nanosuit_obj->SetTranslation(glm::vec3(5.0f, 2.0f, 0.0f));

    std::cout << "Loading light_bulb ..." << std::endl;
    // std::cin.ignore();
    std::shared_ptr<ModelObject> light_bulb_obj(
        ObjectFactory::CreateModelObject(
            "../data/objects/light_bulb/light_bulb.obj"));
    light_bulb_obj->SetScale(glm::vec3(0.1f));
    light_bulb_obj->SetTranslation(glm::vec3(4.0f, -3.0f, 2.0f));

    std::cout << "Loading moon ..." << std::endl;
    std::shared_ptr<ModelObject> moon_obj(
        ObjectFactory::CreateModelObject(
            "../data/objects/moon/moon.obj"));
    moon_obj->SetScale(glm::vec3(0.05f));
    moon_obj->SetTranslation(glm::vec3(7.0f, 3.0f, 3.0f));

    std::cout << "Loading islands ..." << std::endl;
    std::shared_ptr<ModelObject> islands_obj(
        ObjectFactory::CreateModelObject(
            "../data/objects/islands/islands.obj"));
    islands_obj->SetScale(glm::vec3(1.0f));
    islands_obj->SetTranslation(glm::vec3(7.0f, -3.0f, 1.0f));

    std::cout << "Loading kitchen ..." << std::endl;
    std::shared_ptr<ModelObject> kitchen_obj(
        ObjectFactory::CreateModelObject(
            "../data/objects/kitchen/kitchen.obj"));
    kitchen_obj->SetScale(glm::vec3(1.0f));
    kitchen_obj->SetTranslation(glm::vec3(2.0f, 3.0f, 1.0f));

    /*

  std::shared_ptr<ModelObject> cyborg_obj(
      ObjectFactory::CreateModelObject(
          "../data/objects/cyborg/cyborg.obj"));
  cyborg_obj->SetScale(glm::vec3(0.8f));
  cyborg_obj->SetTranslation(glm::vec3(6.0f, 1.0f, 0.0f));

  std::shared_ptr<ModelObject> planet_obj(
      ObjectFactory::CreateModelObject(
          "../data/objects/planet/planet.obj"));
  // planet_obj->SetScale(glm::vec3(0.8f));
  planet_obj->SetTranslation(glm::vec3(10.0f, -5.0f, 6.0f));

  std::shared_ptr<ModelObject> rock_obj(
      ObjectFactory::CreateModelObject(
          "../data/objects/rock/rock.obj"));
  rock_obj->SetScale(glm::vec3(0.3f));
  rock_obj->SetTranslation(glm::vec3(5.0f, -3.0f, 0.0f));
  */

    // std::cout << "Floor = " << floor_obj << std::endl;
    // std::cout << "Camera = " << camera_obj << std::endl;
    // std::cout << "Zero Cube = " << zero_cube_obj << std::endl;
    std::cout << "Debug Cube = " << debug_cube_obj << std::endl;
    // std::cout << "Nanosuit = " << nanosuit_obj << std::endl;

    // std::cout << "Moon = " << moon_obj << std::endl;

    /*
  std::cout << "Nanosuit = " << nanosuit_obj << std::endl;
  std::cout << "Cyborg = " << cyborg_obj << std::endl;
  std::cout << "Planet = " << planet_obj << std::endl;
  std::cout << "Rock = " << rock_obj << std::endl;
  */

    while (gl_window.IsRunning())
    {
        // std::cout << "delta_time = " << gl_window.delta_time << std::endl;

        /* ====================== Render ===================== */
        renderer->Draw(floor_obj);
        renderer->Draw(camera_obj);
        renderer->Draw(zero_cube_obj);
        renderer->Draw(debug_cube_obj);

        renderer->Draw(axes_obj);

    
        // renderer->Draw(nanosuit_obj);
        renderer->Draw(light_bulb_obj);
        renderer->Draw(moon_obj);

        renderer->Draw(islands_obj);
        renderer->Draw(kitchen_obj);

        /*
    renderer->Draw(cyborg_obj);
    renderer->Draw(planet_obj);
    renderer->Draw(rock_obj);
    */

        gl_window.RunLoop();
    }

    return EXIT_SUCCESS;
}
