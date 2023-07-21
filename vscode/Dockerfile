FROM robotlocomotion/drake:jammy-20230518

RUN apt-get -q update && apt-get -q install -y --no-install-recommends nginx-light xvfb && apt-get -q clean

ENV DISPLAY=:1

# Unclear why, but Deepnote may "forget" about this from Drake's
# own Dockerfile?
# https://github.com/RobotLocomotion/drake/blob/v1.15.0/tools/install/dockerhub/jammy/Dockerfile#L17-L18
ENV PATH="/opt/drake/bin:${PATH}" \
  PYTHONPATH="/opt/drake/lib/python3.10/site-packages:${PYTHONPATH}"

RUN pip install ipywidgets==7.7.0 underactuated==2023.5.29

ENTRYPOINT [ "/work/init.sh" ]