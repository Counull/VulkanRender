#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(location = 0) in vec3 fragColor;
layout(location = 1) in vec3 fragNormal;
layout(location = 2) in vec2 fragTexCoord;
layout(location = 3) in vec3 worldPos;

layout(location = 0) out vec4 outColor;
layout(binding = 1) uniform sampler2D texSampler;



void main() {



//  outColor =   vec4( fragColor* texture(texSampler,fragTexCoord).rgb ,1.0f);
//outColor =vec4(fragNormal,1.0f);


float gloss=20;

vec3 cameraPos = vec3(2.0,2.0,2.0);
vec3 light = normalize(vec3(0.5,-1,1));
vec3 lightColor =vec3(1,1,1);


vec3 albedo = fragColor* texture(texSampler,fragTexCoord).rgb;//vec3(1,1,1) ;


vec3 ambient = vec3(0.05,0.05,0.05)*albedo;

vec3 worldNormal = normalize(fragNormal);




vec3 diffuse =  lightColor * albedo *  max(dot(worldNormal,light),0) ; 

vec3 viewDir = normalize(cameraPos -worldPos);
vec3 halfDir = normalize(light+viewDir);
vec3 specular = lightColor *pow(max(0,dot(worldNormal,halfDir)),gloss);

outColor = vec4(ambient+diffuse+specular,1.0f);
//outColor = vec4(ambient+diffuse,1.0f);

}