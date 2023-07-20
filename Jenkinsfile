pipeline {
  agent {
    label "ros && docker && small"
  }
  options {
    buildDiscarder(logRotator(daysToKeepStr: '14', numToKeepStr: '10'))
  }
  stages {
    stage('test') {
      parallel {
        stage('flake8') {
          steps {
            withDockerRegistry(url: "https://242567060652.dkr.ecr.us-west-2.amazonaws.com", credentialsId:"ecr:us-west-2:CIUser") {
              withDockerContainer(image: "242567060652.dkr.ecr.us-west-2.amazonaws.com/ros-ci/flake8:v2.2") {
                sh label: 'Verify python code with flake8', script: "badger-flake8"
              }
            }
          }
          post {
            always {
              junit testResults: 'results/flake8_junit.xml', allowEmptyResults: true
            }
          }
        }
        stage('cpplint') {
          steps {
            withDockerRegistry(url: "https://242567060652.dkr.ecr.us-west-2.amazonaws.com", credentialsId:"ecr:us-west-2:CIUser") {
              withDockerContainer(image: "242567060652.dkr.ecr.us-west-2.amazonaws.com/ros-ci/cpplint:v1.1") {
                sh label: 'Verify c++ code with cpplint', script: "badger-cpplint"
              }
            }
          }
          post {
            always {
              junit 'results/cpplint_junit.xml'
            }
          }
        }
      }
    }
  }
  post {
    always {
      archiveArtifacts artifacts: 'results/*'
    }
  }
}
