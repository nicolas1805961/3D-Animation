
#include "sphere_collision.hpp"

#include <random>

#ifdef SCENE_SPHERE_COLLISION

using namespace vcl;


void scene_model::frame_draw(std::map<std::string,GLuint>& , scene_structure& scene, gui_structure& )
{
    float dt = 0.02f * timer.scale;
    timer.update();

    set_gui();

    create_new_particle();
    compute_time_step(dt);

    display_particles(scene);
    draw(borders, scene.camera);

}

void scene_model::compute_time_step(float dt)
{
    // Set forces
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
        particles[k].f = vec3(0,-9.81f,0);


    // Integrate position and speed of particles through time
    for(size_t k=0; k<N; ++k) {
        particle_structure& particle = particles[k];
        vec3& v = particle.v;
        vec3& p = particle.p;
        vec3 const& f = particle.f;

        v = (1-0.9f*dt) * v + dt * f; // gravity + friction force
        p = p + dt * v;
    }

    // Collisions between spheres
    // ... to do
    for (size_t i = 0; i < N; i++)
    {
        particle_structure& particle1 = particles[i];
        for (size_t j = 0; j < N; j++)
        {
            if (i == j)
                continue;
            particle_structure& particle2 = particles[j];
            vec3 p_prev1 = particle1.p - particle1.v * dt;
            vec3 p_prev2 = particle2.p - particle2.v * dt;
            if (norm(particle1.p - particle2.p) < particle1.r + particle2.r)
            {
                vec3 u1 = (particle1.p - particle2.p) / norm(particle1.p - particle2.p);
                vec3 u2 = (particle2.p - particle1.p) / norm(particle2.p - particle1.p);
                float d = particle1.r + particle2.r - norm(particle1.p - particle2.p);
                particle1.p = particle1.p + (d / 2) * u1;
                particle2.p = particle2.p + (d / 2) * u2;
                particle1.v = 0.6 * particle1.v - 0.6 * ((dot(particle1.v - particle2.v, u1) * u1));
                particle2.v = 0.6 * particle2.v - 0.6 * ((dot(particle2.v - particle1.v, u2) * u2));
            }
        }
    }

    // Collisions with cube
    // ... to do
    for (size_t i = 0; i < N; i++)
    {
        particle_structure& particle = particles[i];

        vec3 point_of_plane_bottom = vcl::vec3(-1, -1, -1);
        vec3 normal_of_plane_bottom = vcl::vec3(0, 1, 0);
        float distance_bottom = dot(particle.p - point_of_plane_bottom, normal_of_plane_bottom);

        vec3 point_of_plane_up = vcl::vec3(-1, 1, -1);
        vec3 normal_of_plane_up = vcl::vec3(0, -1, 0);
        float distance_up = dot(particle.p - point_of_plane_up, normal_of_plane_up);

        vec3 point_of_plane_left = vcl::vec3(-1, 1, -1);
        vec3 normal_of_plane_left = vcl::vec3(1, 0, 0);
        float distance_left = dot(particle.p - point_of_plane_left, normal_of_plane_left);

        vec3 point_of_plane_right = vcl::vec3(1, 1, -1);
        vec3 normal_of_plane_right = vcl::vec3(-1, 0, 0);
        float distance_right = dot(particle.p - point_of_plane_right, normal_of_plane_right);

        vec3 point_of_plane_back = vcl::vec3(-1, 1, -1);
        vec3 normal_of_plane_back = vcl::vec3(0, 0, 1);
        float distance_back = dot(particle.p - point_of_plane_back, normal_of_plane_back);

        vec3 point_of_plane_front = vcl::vec3(-1, -1, 1);
        vec3 normal_of_plane_front = vcl::vec3(0, 0, -1);
        float distance_front = dot(particle.p - point_of_plane_front, normal_of_plane_front);
        if (distance_bottom <= particle.r)
        {
            vec3& normal = vec3(0, 1, 0);
            float d = particle.r - distance_bottom;
            particle.p = particle.p + d * normal;
            vec3 v_parallel = particle.v - dot(particle.v, normal) * normal;
            vec3 v_orthogonal = dot(particle.v, normal) * normal;
            particle.v = 0.6 * v_parallel - 0.6 * v_orthogonal;
        }
        if (distance_up <= particle.r)
        {
            vec3 normal = vec3(0, -1, 0);
            vec3 v_parallel = particle.v - dot(particle.v, normal) * normal;
            vec3 v_orthogonal = dot(particle.v, normal) * normal;
            particle.v = 0.6 * v_parallel - 0.6 * v_orthogonal;
            float d = particle.r - distance_up;
            particle.p = particle.p + d * normal;
        }
        if (distance_left <= particle.r)
        {
            vec3 normal = vec3(1, 0, 0);
            vec3 v_parallel = particle.v - dot(particle.v, normal) * normal;
            vec3 v_orthogonal = dot(particle.v, normal) * normal;
            particle.v = 0.6 * v_parallel - 0.6 * v_orthogonal;
            float d = particle.r - distance_left;
            particle.p = particle.p + d * normal;
        }
        if (distance_right <= particle.r)
        {
            vec3 normal = vec3(-1, 0, 0);
            vec3 v_parallel = particle.v - dot(particle.v, normal) * normal;
            vec3 v_orthogonal = dot(particle.v, normal) * normal;
            particle.v = 0.6 * v_parallel - 0.6 * v_orthogonal;
            float d = particle.r - distance_right;
            particle.p = particle.p + d * normal;
        }
        if (distance_back <= particle.r)
        {
            vec3 normal = vec3(0, 0, 1);
            vec3 v_parallel = particle.v - dot(particle.v, normal) * normal;
            vec3 v_orthogonal = dot(particle.v, normal) * normal;
            particle.v = 0.6 * v_parallel - 0.6 * v_orthogonal;
            float d = particle.r - distance_back;
            particle.p = particle.p + d * normal;
        }
        if (distance_front <= particle.r)
        {
            vec3 normal = vec3(0, 0, -1);
            vec3 v_parallel = particle.v - dot(particle.v, normal) * normal;
            vec3 v_orthogonal = dot(particle.v, normal) * normal;
            particle.v = 0.6 * v_parallel - 0.6 * v_orthogonal;
            float d = particle.r - distance_front;
            particle.p = particle.p + d * normal;
        }
    } 
}


void scene_model::create_new_particle()
{
    // Emission of new particle if needed
    timer.periodic_event_time_step = gui_scene.time_interval_new_sphere;
    const bool is_new_particle = timer.event;
    static const std::vector<vec3> color_lut = {{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1}};

    if( is_new_particle && gui_scene.add_sphere)
    {
        particle_structure new_particle;

        new_particle.r = 0.08f;
        new_particle.c = color_lut[int(rand_interval()*color_lut.size())];

        // Initial position
        new_particle.p = vec3(0,0,0);

        // Initial speed
        const float theta = rand_interval(0, 2*3.14f);
        new_particle.v = vec3( 2*std::cos(theta), 5.0f, 2*std::sin(theta));

        particles.push_back(new_particle);

    }
}
void scene_model::display_particles(scene_structure& scene)
{
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
    {
        const particle_structure& part = particles[k];

        sphere.uniform.transform.translation = part.p;
        sphere.uniform.transform.scaling = part.r;
        sphere.uniform.color = part.c;
        draw(sphere, scene.camera);
    }
}

void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{
    sphere = mesh_drawable( mesh_primitive_sphere(1.0f));
    sphere.shader = shaders["mesh"];

    std::vector<vec3> borders_segments = {{-1,-1,-1},{1,-1,-1}, {1,-1,-1},{1,1,-1}, {1,1,-1},{-1,1,-1}, {-1,1,-1},{-1,-1,-1},
                                          {-1,-1,1} ,{1,-1,1},  {1,-1,1}, {1,1,1},  {1,1,1}, {-1,1,1},  {-1,1,1}, {-1,-1,1},
                                          {-1,-1,-1},{-1,-1,1}, {1,-1,-1},{1,-1,1}, {1,1,-1},{1,1,1},   {-1,1,-1},{-1,1,1}};
    borders = segments_gpu(borders_segments);
    borders.uniform.color = {0,0,0};
    borders.shader = shaders["curve"];

}



void scene_model::set_gui()
{
    // Can set the speed of the animation
    ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
    ImGui::SliderFloat("Interval create sphere", &gui_scene.time_interval_new_sphere, 0.05f, 2.0f, "%.2f s");
    ImGui::Checkbox("Add sphere", &gui_scene.add_sphere);

    bool stop_anim  = ImGui::Button("Stop"); ImGui::SameLine();
    bool start_anim = ImGui::Button("Start");

    if(stop_anim)  timer.stop();
    if(start_anim) timer.start();
}





#endif
