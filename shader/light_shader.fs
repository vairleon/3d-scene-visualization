#version 330 core

out vec4 FragColor;

struct Material {
    //sampler2D diffuse;
    //sampler2D specular;    
    float shininess;
}; 

struct Light {
    //vec3 position;
    vec3 direction;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    vec3 lightColor;
};


in vec3 vertexColor;
in vec3 Normal;
in vec3 FragPos;  


uniform float alpha;
uniform vec3 viewPos;
uniform Material material;
uniform Light light;
uniform bool blinn;
uniform bool if_setColor;
uniform vec3 objectColor;

void main()
{
    // ambient
    vec3 ambient = light.ambient * light.lightColor; //* texture(material.diffuse, TexCoords).rgb;
  	
    // diffuse 
    vec3 norm = normalize(Normal);
    // vec3 lightDir = normalize(light.position - FragPos);
    vec3 lightDir = normalize(-light.direction);  
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = light.diffuse * light.lightColor * diff; //* texture(material.diffuse, TexCoords).rgb;  

    // specular
    //vec3 lightDir = normalize(lightPos - FragPos);
    vec3 viewDir = normalize(viewPos - FragPos);

    float spec = 0.0;
    if(blinn)
    {
        vec3 halfwayDir = normalize(lightDir + viewDir);  
        spec = pow(max(dot(norm, halfwayDir), 0.0), material.shininess*4);
    }
    else
    {
        vec3 reflectDir = reflect(-lightDir, norm);
        spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    }
    
    //vec3 halfwayDir = normalize(lightDir + viewDir);
    //float spec = pow(max(dot(viewDir, reflectDir), 0.0), light.shininess);
    vec3 specular = light.specular * spec ;  //* texture(material.specular, TexCoords).rgb; 
	
    vec3 result = (ambient + diffuse + specular) * vertexColor;
    
    if(if_setColor)
    {
        result = (ambient + diffuse + specular) * objectColor;
	}
   
    FragColor = vec4(result, alpha);
} 

