shader_type spatial;
render_mode diffuse_lambert, specular_disabled;

uniform vec3 base_color : source_color = vec3(0.6, 0.6, 0.6);
uniform float vsize = 0.2;          // world-space block size (metres)
uniform float line_frac = 0.06;     // grid-line half-width as fraction of a block
uniform float line_dark = 0.5;      // brightness at a block boundary line
uniform float ao_frac = 0.32;       // block-edge AO falloff (fraction of block)
uniform float ao_dark = 0.72;       // brightness at the very edge (fake AO)

varying vec3 w_pos;
varying vec3 w_nrm;

void vertex() {
    w_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
    w_nrm = normalize((MODEL_MATRIX * vec4(NORMAL, 0.0)).xyz);
}

void fragment() {
    vec3 g = fract(w_pos / vsize);
    vec3 d = min(g, 1.0 - g);              // 0..0.5 distance to nearest boundary per axis
    vec3 inplane = 1.0 - abs(w_nrm);       // ~0 for the face-normal axis, ~1 for in-plane axes
    // nearest in-plane boundary distance (ignore the normal axis by pushing it to 1.0)
    float dist = 1.0;
    dist = min(dist, mix(1.0, d.x, inplane.x));
    dist = min(dist, mix(1.0, d.y, inplane.y));
    dist = min(dist, mix(1.0, d.z, inplane.z));
    float line = mix(line_dark, 1.0, smoothstep(0.0, line_frac, dist));   // dark line at boundary
    float ao   = mix(ao_dark, 1.0, smoothstep(0.0, ao_frac, dist));        // soft AO toward edges
    ALBEDO = base_color * line * ao;
    METALLIC = 0.0;
    ROUGHNESS = 1.0;
}
