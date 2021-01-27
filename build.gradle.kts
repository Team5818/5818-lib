plugins {
    `java-library`
    id("com.techshroom.incise-blue") version "0.5.7"
    id("net.researchgate.release") version "2.8.1"
    id("com.jfrog.bintray") version "1.8.4"
    `maven-publish`
}

inciseBlue {
    ide()
    license()
    util {
        javaVersion = JavaVersion.VERSION_11
    }
}

repositories {
    maven {
        name = "WPI Maven"
        url = uri("https://frcmaven.wpi.edu/artifactory/release")
    }
    maven {
        name = "CTRE Maven"
        url = uri("https://devsite.ctr-electronics.com/maven/release/")
    }
}

dependencies {
    val wpiVersion = "2020.3.2"
    api("edu.wpi.first.wpilibj:wpilibj-java:$wpiVersion")
    api("edu.wpi.first.ntcore:ntcore-java:$wpiVersion")
    api("edu.wpi.first.cscore:cscore-java:$wpiVersion")
    api("edu.wpi.first.cameraserver:cameraserver-java:$wpiVersion")
    api("edu.wpi.first.wpilibNewCommands:wpilibNewCommands-java:$wpiVersion")

    val ctreVersion = "5.19.4"
    api("com.ctre.phoenix:wpiapi-java:$ctreVersion")
    api("com.ctre.phoenix:api-java:$ctreVersion")
}

release {
    tagTemplate = "v\${version}"
    buildTasks = listOf("build")
}

tasks.named("afterReleaseBuild") {
    dependsOn("bintrayUpload")
}

java.withJavadocJar()
java.withSourcesJar()

publishing {
    publications {
        register<MavenPublication>("library") {
            pom {
                name.set("5818-lib")
            }
            groupId = "org.rivierarobotics"
            artifactId = "5818-lib"
            version = project.version.toString()

            from(components["java"])
        }
    }
}

bintray {
    user = System.getenv("BINTRAY_USER") ?: findProperty("bintray.user")?.toString()
    key = System.getenv("BINTRAY_KEY") ?: findProperty("bintray.password")?.toString()
    setPublications("library")
    with(pkg) {
        repo = "maven-release"
        name = "5818-lib"
        userOrg = "team5818"
        vcsUrl = "https://github.com/Team5818/5818-lib.git"
        publish = true
        setLicenses("GPL-3.0-or-later")
        with(version) {
            name = project.version.toString()
        }
    }
}