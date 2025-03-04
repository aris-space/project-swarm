Workflow for implementing a new feature:

1) Setup:
- Always begin by working from the develop branch.
- Create a separate branch for the new feature to keep changes isolated.

2) Developing the Feature:
- Implement the feature and test it thoroughly.
- Make regular commits with clear descriptions of changes.
- Make sure that everything is properly commented, so that the reviewers and future you understand the code
- Document non-code related information in Notion, e.g. hardware setup guides, wiring charts, configuration steps etc.
- Push the branch to the remote repository to ensure changes are tracked.


3) Opening a Pull Request:
- Once the feature is working, create a pull request to merge the feature branch into develop.
- At least two team members must review and approve the pull request.
- Address any feedback and make necessary improvements.

3) Merging into Develop:
- After approval, merge the feature branch into develop.



Merging into Main for Integration:
- When the develop branch reaches a stable and fully integrated state, it is merged into the main branch.
- If applicable, tag the new version and prepare for deployment.
- Update any relevant documentation.
