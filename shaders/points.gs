#version 330 core
layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in VS_OUT {
    vec4 color;
    vec4 color_tl;
    vec4 color_tr;
    vec4 color_bl;
    vec4 color_br;
} gs_in[];

// in vec4 vColor;

out vec4 VertexColor;



void make_rect(vec4 position, float size)
{    
    // fColor = gs_in[0].color; // gs_in[0] since there's only one input vertex

    // VertexColor = gs_in[0].color;
    // gl_Position = position; // 0: center
    // EmitVertex();   

    VertexColor = gs_in[0].color_br;
    gl_Position = position + vec4(size, -size, 0.0, 0.0); // 2:bottom-right
    EmitVertex();

    VertexColor = gs_in[0].color_tr;
    gl_Position = position + vec4(size, size, 0.0, 0.0); // 3:top-right
    EmitVertex();

    VertexColor = gs_in[0].color_bl;
    gl_Position = position + vec4(-size, -size, 0.0, 0.0); // 1:bottom-left
    EmitVertex();    

    VertexColor = gs_in[0].color_tl;
    gl_Position = position + vec4(-size, size, 0.0, 0.0); // 4:top-left
    EmitVertex();

    EndPrimitive();
}

void main() {    
    // make_rect(gl_in[0].gl_Position, 0.15);
    make_rect(gl_in[0].gl_Position, 0.15);
}